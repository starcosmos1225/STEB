#! /usr/bin/env python

from robot import Robot, ACNet
#from DoubleLayerRobot import DoubleLayerRobot, ACNet
import time
import threading
import rospy
import numpy as np
import tensorflow as tf
import subprocess
import signal
import os
import sys
from genRandomWorld import genRandomWorld
from fileUpLoad import fileUpLoad, fileDownLoad
from Env import Interact
import DoubleLayerEnv as dl
MAX_EP = 10000
SAVE_FREQUENT = 10
#HOST_IP = "192.168.1.102"
#USER_NAME= "zhang"
#PASSWORD = "Wozuike2"
#REMOTE_PATH = "E:/tool"
#HOST_IP = "192.168.1.104"
#USER_NAME= "huxingyu"
#PASSWORD = "250338"
#REMOTE_PATH = "~/service"
#HOST_IP = "192.168.31.190"
#USER_NAME= "hxj"
#PASSWORD = "250338"
#REMOTE_PATH = "~/service"
#HOST_IP = "192.168.31.149"
#USER_NAME= "zhy"
#PASSWORD = "123"
#REMOTE_PATH = "/home/zhy/catkin_ws/src/robottest/service"
HOST_IP = "127.0.0.1"
USER_NAME= "hxj"
PASSWORD = "250338"
REMOTE_PATH = "/home/hxj/tool/ros_husky_ws/src/robottest/service"
class MultiRobot:
    def __init__(self, n, worlds):
        self.sess = tf.Session()
        self.worlds = worlds# cube_number,W,H,offsetX,offsetY
        self.globalNet = ACNet(self.sess, "GLOBAL", 5, 3)
        self.sess.run(tf.global_variables_initializer())
        self.n = n
        self.robotlist = []
        self.child = None
        useTrainedNet = True
        #self.globalNet.load()
        #try:
            #useTrainedNet = self.download()
            #if useTrainedNet:
                #self.globalNet.load()
        #except Exception:
            #print("download fail!")
            #useTrainedNet = False
        #print(n)
        for i in range(n):
            #robot = DoubleLayerRobot('h'+str(i+1), self.globalNet, useTrainedNet)
            robot = Robot('h' + str(i + 1), self.sess,self.globalNet, useTrainedNet)
            self.robotlist.append(robot)
        #for robot in self.robotlist:
            #print(robot.name)
        #t=input()
        pass

    def reset(self):
        #first kill process gzserver
        if self.child is not None:
            # subprocess.Popen(["rosservice", "call", "/gazebo/delete_model", "model_name: '/'"])
            time.sleep(0.1)
            self.child.send_signal(signal.SIGINT)
            time.sleep(5)
            subprocess.Popen(["killall", "gzserver"])
            subprocess.Popen(["killall", "gazebo"])
            time.sleep(0.1)
        # roslaunch husky_gazebo husky_RL.launch x:=<location X> y:=<location Y>
        for robot in self.robotlist:
            robot.env = Interact(robot.name, 5)
            #env = Interact(robot.name, 5)
            #robot.env = dl.Interact(env, action_n=2, mapsize=400)
        path = os.path.split(os.path.realpath(__file__))[0]
        path += "/../worlds/random.world"
        # path += "/../worlds/empty.world"
        locations = genRandomWorld(path, self.worlds)
        for robot, location in zip(self.robotlist, locations):
            robot.initEnv(location)
        args = ["roslaunch", "robottest", "RL_Env.launch"]
        for i in range(self.n):
            x = "x"+ str(i+1)+":="+str(locations[i][0])
            y = "y"+ str(i+1)+":="+str(locations[i][1])
            args.append(x)
            args.append(y)
        time.sleep(1.0)
        #print("begin up")
        self.child = subprocess.Popen(args)
        #print("end up")
        time.sleep(5.0)
        pass

    def train(self):
        ep = 0
        while True:
            #setup a roslaunch ENV
            ep += 1
            print("MULTI:{}".format(ep))
            self.reset()
            #print("end reset")
            COORD = tf.train.Coordinator()
            worker_threads = []
            for robot in self.robotlist:
                print("robot name:{}".format(robot.name))
                #job = lambda: robot.trainOnce()
                t = threading.Thread(target=robot.trainOnce)
                worker_threads.append(t)
            for thread in worker_threads:
                thread.setDaemon(True)
                thread.start()
            COORD.join(worker_threads)
            if ep % SAVE_FREQUENT == 0:
                self.globalNet.save()
                #self.upload()
            if ep > MAX_EP:
                break
        pass
    def test(self):
        ep = 0
        while True:
            #setup a roslaunch ENV
            ep += 1
            print("MULTI:{}".format(ep))
            self.reset()
            #print("end reset")
            COORD = tf.train.Coordinator()
            worker_threads = []
            for robot in self.robotlist:
                print("robot name:{}".format(robot.name))
                #job = lambda: robot.trainOnce()
                t = threading.Thread(target=robot.test)
                worker_threads.append(t)
            for thread in worker_threads:
                thread.setDaemon(True)
                thread.start()
            COORD.join(worker_threads)
            if ep > MAX_EP:
                break
            break
        pass

    def download(self):
        fileDownLoad(filename='/model.ckpt.meta', host_ip=HOST_IP, username=USER_NAME,
            password=PASSWORD, remotepath=REMOTE_PATH+'/model.ckpt.meta')
        fileDownLoad(filename='/model.ckpt.index', host_ip=HOST_IP, username=USER_NAME,
            password=PASSWORD, remotepath=REMOTE_PATH+'/model.ckpt.index')
        fileDownLoad(filename='/model.ckpt.data-00000-of-00001', host_ip=HOST_IP, username=USER_NAME,
            password=PASSWORD, remotepath=REMOTE_PATH+'/model.ckpt.data-00000-of-00001')
        fileDownLoad(filename='/checkpoint', host_ip=HOST_IP, username=USER_NAME,
            password=PASSWORD, remotepath=REMOTE_PATH+'/checkpoint')
    def upload(self):
        # upload the model
        # the name is /model_upload.***
        #######################################################
        fileUpLoad(filename='/model.ckpt.meta', host_ip=HOST_IP, username=USER_NAME,
            password=PASSWORD, remotepath=REMOTE_PATH + '/model.ckpt.meta')
        fileUpLoad(filename='/model.ckpt.index', host_ip=HOST_IP, username=USER_NAME,
            password=PASSWORD, remotepath=REMOTE_PATH + '/model.ckpt.index')
        fileUpLoad(filename='/model.ckpt.data-00000-of-00001', host_ip=HOST_IP, username=USER_NAME,
            password=PASSWORD, remotepath=REMOTE_PATH + '/model.ckpt.data-00000-of-00001')
        #fileUpLoad(filename='/checkpoint', host_ip=HOST_IP, username=USER_NAME,
            #password=PASSWORD, remotepath=REMOTE_PATH + '/checkpoint')

def init():
    rospy.init_node('mainController', anonymous=True)
    worlds = [[20, 20, 20, 0, 0]]# cube_number,W,H,offsetX,offsetY
    #worlds = [[20, 20, 20, 0, 0],
              #[20, 20, 20, 30, 0],
              #[20, 20, 20, 60, 0]]
    robots = MultiRobot(len(worlds), worlds)
    print("begin trian")
    robots.train()
    rospy.spin()
def test():
    rospy.init_node('mainController', anonymous=True)
    worlds = [[20, 20, 20, 0, 0]]# cube_number,W,H,offsetX,offsetY
    robots = MultiRobot(len(worlds), worlds)
    print("begin trian")
    robots.test()
    rospy.spin()
if __name__=='__main__':
    test()