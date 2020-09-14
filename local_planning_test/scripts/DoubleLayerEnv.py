#! /usr/bin/env python
import numpy as np
import math
import rospy
from actionlib_msgs.msg import GoalID
from robot import ACNet
import tensorflow as tf
from Vec import Vec2d
class Interact:
    def __init__(self, env, action_n=2, mapsize=400):
        self.env = env
        self.action_n = action_n
        g2 = tf.Graph()
        self.sess = tf.Session(graph=g2)
        with self.sess.as_default():
            with self.sess.graph.as_default():
                self.GlobalNet = ACNet(self.sess, "GLOBAL", 5, 3, graph=g2)
                self.sess.run(tf.global_variables_initializer())
                self.GlobalNet.load()
                self.ACNet = ACNet(self.sess, "Local", 5, 3,globalAC=self.GlobalNet)
                self.ACNet.pull_global()
        self.state = [None]*5
        self.size = mapsize
        self.action_list = [0, 1]
        self.oldAction = None

    def getState(self):
        self.state = self.env.getState()
        self.state[4] = self.env.getGlobalMap()
        return np.array(self.state)

    def clearState(self):
        self.state = [None] * 5
        self.env.clearState()

    def step(self, action):
        done = False
        info = None
        if action == 0:
            #movebase
            if self.oldAction == None or self.oldAction == 1:
                self.env.sendGoal()
                self.oldAction = 0
            RL_step = 0
            end = False
            ep_r = 0.0
            while not end:
                rospy.sleep(0.1)
                RL_step += 1
                s_, r, done, info = self.env.getStatus()
                #print(r)
                ep_r += r
                if done or RL_step > 80:
                    end = True
        else:
            #stop move_base
            if self.oldAction == None or self.oldAction == 0:
                self.oldAction = 1
                msg = GoalID()
                self.env.pubStopMovebase.publish(msg)
            RL_step = 0
            end = False
            ep_r = 0.0
            while not end:
                s = self.env.getState()
                a = self.ACNet.choose_action(s)  # Actor
                self.env.step(a)
                RL_step += 1
                s_, r, done, info = self.env.getStatus()
                ep_r += r
                if done or RL_step > 80:
                    end = True
        return self.getState(), ep_r, done, info

    def getAction(self):
        return self.action_n

    def ready(self):
        return self.env.ready()

    def trans2robotTf(self, location):
        carX,carY,goalX,goalY = location
        carPose = Vec2d(self.env.initPose[0, 0], self.env.initPose[0, 1])
        carPosition = Vec2d(carX, carY)
        goalPosition = Vec2d(goalX, goalY)
        worldPose =Vec2d(1, 0)#world's x axis
        GC = goalPosition-carPosition
        costheta = worldPose.dot(carPose)
        sintheta = carPose.cross(worldPose)
        rotate = np.mat([[costheta, -sintheta], [sintheta, costheta]])
        goal = np.mat([[GC.x], [GC.y]])
        robotTfGoal = rotate*goal
        return carX, carY, robotTfGoal[0, 0], robotTfGoal[1, 0]

    def initEnv(self, location):
        self.env.position.x, self.env.position.y, self.env.goal.x, self.env.goal.y = self.trans2robotTf(location)
        self.env.receiveGoal = True
        print("location:{} {}".format(self.env.position.x, self.env.position.y))
        print("goal:{} {}".format(self.env.goal.x, self.env.goal.y))
        self.env.clearState()
        pass
