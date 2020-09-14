#! /usr/bin/env python
import rospy
import numpy as np
import tensorflow as tf
import random
import math
#from Env import Interact
import os
import sys
from Vec import Vec2d
np.random.seed(2)
tf.set_random_seed(2)  # reproducible

MAX_EPISODE = 2000
DISPLAY_REWARD_THRESHOLD = 200
LEARN_STEPS = 100
MAX_EP_STEPS = 1500
RENDER = False
GAMMA = 0.9
LR_A = 0.001  # Actor
LR_C = 0.001  # Critic
ENTROPY_BETA = 0.001
N_F = 5#env.observation_space.shape[0]
N_A = 2#0 means choose move_base 1 means choose robot
GLOBAL_NET_SCOPE = "DLGLOBAL"

class ACNet(object):
    def __init__(self, sess, scope, n_features, n_actions, globalAC=None):
        self.sess = sess
        self.n_a = n_actions
        self.n_s = n_features
        if scope == GLOBAL_NET_SCOPE:   # get global network
            with tf.variable_scope(scope):
                self.s = tf.placeholder(tf.float32, [None, n_features-1], 'S')
                self.image = tf.placeholder(tf.float32, [None, 200, 200, 1], 'IMAGE')
                self.a_params, self.c_params = self._build_net(scope)[-2:]
                self.saver = tf.train.Saver()
        else:   # local net, calculate losses
            with tf.variable_scope(scope):
                self.s = tf.placeholder(tf.float32, [None, n_features-1], 'S')
                self.image = tf.placeholder(tf.float32, [None, 200, 200, 1], 'IMAGE')
                self.a_his = tf.placeholder(tf.int32, [None, ], 'A')
                self.v_target = tf.placeholder(tf.float32, [None, 1], 'Vtarget')
                self.a_prob, self.v, self.a_params, self.c_params = self._build_net(scope)
                td = tf.subtract(self.v_target, self.v, name='TD_error')
                with tf.name_scope('c_loss'):
                    self.c_loss = tf.reduce_mean(tf.square(td))
                with tf.name_scope('a_loss'):

                    log_prob = tf.reduce_sum(tf.log(self.a_prob + 1e-5) * tf.one_hot(self.a_his, n_actions, dtype=tf.float32), axis=1, keep_dims=True)
                    exp_v = log_prob * tf.stop_gradient(td)
                    entropy = -tf.reduce_sum(self.a_prob * tf.log(self.a_prob + 1e-5),
                                             axis=1, keep_dims=True)  # encourage exploration
                    self.exp_v = ENTROPY_BETA * entropy + exp_v
                    self.a_loss = tf.reduce_mean(-self.exp_v)

                with tf.name_scope('local_grad'):
                    self.a_grads = tf.gradients(self.a_loss, self.a_params)
                    self.c_grads = tf.gradients(self.c_loss, self.c_params)

            with tf.name_scope('sync'):
                with tf.name_scope('pull'):
                    self.pull_a_params_op = [l_p.assign(g_p) for l_p, g_p in zip(self.a_params, globalAC.a_params)]
                    self.pull_c_params_op = [l_p.assign(g_p) for l_p, g_p in zip(self.c_params, globalAC.c_params)]
                with tf.name_scope('push'):
                    self.update_a_op = tf.train.RMSPropOptimizer(LR_A, name='RMSPropA').apply_gradients(zip(self.a_grads, globalAC.a_params))
                    self.update_c_op = tf.train.RMSPropOptimizer(LR_C, name='RMSPropC').apply_gradients(zip(self.c_grads, globalAC.c_params))

    def _build_net(self, scope):
        w_init = tf.random_normal_initializer(0., .1)
        with tf.variable_scope('actor'):
            # 200*200
            image_l1 = tf.layers.conv2d(inputs=self.image, filters=6, kernel_size=5, strides=(1,1),padding = 'valid',
                                        activation=tf.nn.relu6, kernel_initializer=w_init)
            image_p1 = tf.layers.max_pooling2d(inputs=image_l1, pool_size=(4, 4), padding='valid',strides=(2,2))
            # 49*49
            image_l2 = tf.layers.conv2d(inputs = image_p1, filters=16, kernel_size=6, strides=(1,1),padding = 'valid',
                                        activation=tf.nn.relu6, kernel_initializer=w_init)

            image_p2 = tf.layers.max_pooling2d(inputs=image_l2, pool_size=(4, 4), padding='valid',strides=(2, 2))
            # 11*11
            image_l3 = tf.layers.conv2d(inputs=image_p2, filters=16, kernel_size=6, strides=(1, 1), padding='valid',
                                        activation=tf.nn.relu6, kernel_initializer=w_init)

            image_p3 = tf.layers.max_pooling2d(inputs=image_l3, pool_size=(2, 2), padding='valid', strides=(2, 2))
            # 3*3
            image_f3 = tf.layers.flatten(image_p3)
            # 3*3*16+4
            s1 = tf.concat([self.s, image_f3], axis=1)
            l_a = tf.layers.dense(s1, 200, tf.nn.relu6, kernel_initializer=w_init, name='la')
            a_prob = tf.layers.dense(l_a, self.n_a, tf.nn.softmax, kernel_initializer=w_init, name='ap')
        with tf.variable_scope('critic'):
            image_l1 = tf.layers.conv2d(inputs=self.image, filters=6, kernel_size=5, strides=(1, 1), padding='valid',
                                        activation=tf.nn.relu6, kernel_initializer=w_init)
            image_p1 = tf.layers.max_pooling2d(inputs=image_l1, pool_size=(4, 4), padding='valid',strides=(2,2))
            image_l2 = tf.layers.conv2d(inputs=image_p1, filters=16, kernel_size=6, strides=(1, 1), padding='valid',
                                        activation=tf.nn.relu6, kernel_initializer=w_init)
            image_p2 = tf.layers.max_pooling2d(inputs=image_l2, pool_size=(4, 4), padding='valid', strides=(2, 2))
            image_l3 = tf.layers.conv2d(inputs=image_p2, filters=16, kernel_size=6, strides=(1, 1), padding='valid',
                                        activation=tf.nn.relu6, kernel_initializer=w_init)

            image_p3 = tf.layers.max_pooling2d(inputs=image_l3, pool_size=(2, 2), padding='valid', strides=(2, 2))
            # 3*3
            image_f3 = tf.layers.flatten(image_p3)
            s1 = tf.concat([self.s, image_f3], axis=1)
            l_c = tf.layers.dense(s1, 100, tf.nn.relu6, kernel_initializer=w_init, name='lc')
            v = tf.layers.dense(l_c, 1, kernel_initializer=w_init, name='v')  # state value
        a_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=scope + '/actor')
        c_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=scope + '/critic')
        return a_prob, v, a_params, c_params
    def update_global(self, feed_dict):  # run by a local
        self.sess.run([self.update_a_op, self.update_c_op], feed_dict)  # local grads applies to global net

    def pull_global(self):  # run by a local
        self.sess.run([self.pull_a_params_op, self.pull_c_params_op])

    def choose_action(self, s):  # run by a local
        prob_weights = self.sess.run(self.a_prob, feed_dict={self.s: s[np.newaxis, :-1],self.image:s[-1][np.newaxis,:]})
        action = np.random.choice(range(prob_weights.shape[1]),
                                  p=prob_weights.ravel())  # select action w.r.t the actions prob
        return action

    def save(self, filename="/doubleLayer/model"):
        path = os.path.split(os.path.realpath(__file__))[0]
        path += filename
        self.saver.save(self.sess, path)
        # pass
    def load(self):
        checkpoint = os.path.split(os.path.realpath(__file__))[0]
        checkpoint = checkpoint + "/doubleLayer"
        ckpt = tf.train.latest_checkpoint(checkpoint)
        path = checkpoint + "/checkpoint"
        if not os.path.exists(path):
            return False
        # saver = tf.train.Saver()
        self.saver.restore(self.sess, ckpt)
        return True
class DoubleLayerRobot:
    def __init__(self, name,globalAC=None,useTrainedNet=False):
        self.name = name
        self.iterator_n = 0
        self.action_n = N_A
        #self.env = Interact(name, 5)
        self.state_n = N_F
        self.sess = tf.Session()
        self.ACNet = ACNet(self.sess, name, self.state_n, self.action_n, globalAC)
        self.sess.run(tf.global_variables_initializer())
        if useTrainedNet:
            self.ACNet.pull_global()
        self.child = None
        self.childNavi = None
        self.env = None
        self.receiveGoal = True
        self.totalStep = 0
        self.buffer_s, self.buffer_image, self.buffer_a, self.buffer_v_target = [], [], [], []
    def trans2robotTf(self,location):
        carX,carY,goalX,goalY = location
        #self.env.initPose
        worldPosition = Vec2d(0, 0)
        carPose = Vec2d(self.env.initPose[0, 0], self.env.initPose[0, 1])
        carPosition = Vec2d(carX, carY)
        goalPosition = Vec2d(goalX, goalY)
        worldPose =Vec2d(1, 0)#world's x axis
        GC = goalPosition-carPosition
        costheta = worldPose.dot(carPose)
        sintheta = carPose.cross(worldPose)
        rotate = np.mat([[costheta, -sintheta], [sintheta, costheta]])
        goal = np.mat([[GC.x], [GC.y]])
        robotTfGoal = rotate * goal
        return carX, carY, robotTfGoal[0, 0], robotTfGoal[1, 0]

    def initEnv(self, location):
        self.env.initEnv(location)
        pass

    def trainOnce(self):
        print("begin train {}".format(self.name))
        buffer_s, buffer_image, buffer_a, buffer_r = [], [], [], []
        ep_r = 0
        c = 0
        while True:
            if self.env.ready() or c > 1000:
                break
            #print("not ready")
            rospy.sleep(0.1)
            c += 1

        if c>1000:
            print("not ready exit!{}".format(self.name))
            return
        print("ready {}".format(self.name))
        step = 0
        while True:

            s = self.env.getState()
            a = self.ACNet.choose_action(s)  # Actor

            s_, r, done, info = self.env.step(1)
            if self.name == 'h1':
                print("name:{} step:{} action:{} reward:{}".format(self.name, step, a,r))
                print("distance:{}".format(s_[0]))
            step += 1
            ep_r += r
            buffer_s.append(s[:-1])
            buffer_image.append(s[-1])
            buffer_a.append(a)
            buffer_r.append(r)
            self.totalStep += 1
            #print("{}:step:{} action:{} distance:{} r:{} total:{}".format(self.name, step, a, s_[0], ep_r,self.totalStep))
            if self.totalStep % LEARN_STEPS == 0:
                self.totalStep = 0
                if done:
                    v_s_ = 0   # terminal
                else:
                    v_s_ = self.sess.run(self.ACNet.v, {self.ACNet.s: s_[np.newaxis, :-1],
                                                            self.ACNet.image: s_[-1][np.newaxis, :]})[0, 0]
                buffer_v_target = []
                for r in buffer_r[::-1]:    # reverse buffer r
                    v_s_ = r + GAMMA * v_s_
                    buffer_v_target.append(v_s_)
                buffer_v_target.reverse()
                self.storage(buffer_s, buffer_image, buffer_a, buffer_v_target)
                #buffer_s, buffer_a, buffer_v_target = np.vstack(buffer_s), np.array(buffer_a), np.vstack(buffer_v_target)
                feed_dict = {
                    self.ACNet.s: np.vstack(self.buffer_s),
                    self.ACNet.image: self.buffer_image,
                    self.ACNet.a_his: np.array(self.buffer_a),
                    self.ACNet.v_target: np.vstack(self.buffer_v_target)
                }
                self.update(feed_dict)
                self.buffer_s, self.buffer_image, self.buffer_a, self.buffer_v_target = [], [], [], []

            if step % MAX_EP_STEPS == 0 or done:
                if done:
                    v_s_ = 0  # terminal
                else:
                    v_s_ = self.sess.run(self.ACNet.v, {self.ACNet.s: s_[np.newaxis, :-1],
                                                            self.ACNet.image: s_[-1][np.newaxis, :]})[0, 0]
                buffer_v_target = []
                for r in buffer_r[::-1]:  # reverse buffer r
                    v_s_ = r + GAMMA * v_s_
                    buffer_v_target.append(v_s_)
                    buffer_v_target.reverse()
                self.storage(buffer_s, buffer_image, buffer_a, buffer_r)
                buffer_s, buffer_image, buffer_a, buffer_r = [], [], [], []
                print("{}:step:{} total:{}".format(self.name, step, self.totalStep))
                print("| Ep_r: %f" % ep_r)
                break
    def test(self):
        print("begin train {}".format(self.name))
        ep_r = 0
        c = 0
        while True:
            if self.env.ready() or c > 1000:
                break
            # print("not ready")
            rospy.sleep(0.1)
            c += 1

        if c > 1000:
            print("not ready exit!{}".format(self.name))
            return
        print("ready {}".format(self.name))
        step = 0
        while True:
            s = self.env.getState()
            print("direction {} theta{}".format(s[0], s[1]))
            a = self.ACNet.choose_action(s)  # Actor
            s_, r, done, info = self.env.step(a)
            step += 1
            ep_r += r
            if step % MAX_EP_STEPS == 0 or done:
                print("{}:step:{} total:{}".format(self.name, step, self.totalStep))
                print("| Ep_r: %f" % ep_r)


    def storage(self,buffer_s, buffer_image, buffer_a, buffer_v_target):
        for s, image, a, r in zip(buffer_s, buffer_image, buffer_a, buffer_v_target):
            self.buffer_s.append(s)
            self.buffer_image.append(image)
            self.buffer_v_target.append(r)
            self.buffer_a.append(a)
    def update(self,data):
        self.ACNet.update_global(data)
        self.ACNet.pull_global()
