#! /usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import Odometry
from Env import Interact
from Vec import Vec2d
import numpy as np
import math
#import tensorflow as tf
pubAction = None
pubStopMovebase = None
env = Interact(5)
goal = Vec2d()
position = Vec2d()
pose = Vec2d()
initPose = np.mat([[1, 0, 0]])
receiveGoal = False
def getPosition(data):
    #print("receive position:{} {}".format(data.pose.pose.position.x,data.pose.pose.position.y))
    position.x = data.pose.pose.position.x
    position.y = data.pose.pose.position.y
    x, y, z, w = data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w
    rotateMat = np.mat([[1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*w*y+2*x*z],
                        [2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x],
                        [2*x*z-2*w*y, 2*w*x+2*y*z, 1-2*x*x-2*y*y]])
    pose_ = initPose*rotateMat
    pose2d = Vec2d(pose_[0,0], pose_[0,1]).norm()
    pose.x = pose2d.x
    pose.y = pose2d.y
    if receiveGoal:
        d = goal-pose
        distance = d.length()
        d = d.norm()
        sintheta = pose.cross(d)
        costheta = d.dot(pose)
        # print(d)
        # print("direction2:{} {}".format(direction2.x,direction2.y))
        # print("direction1:{} {}".format(direction1.x, direction1.y))
        # t=input()
        if sintheta > 0:
            theta = 2 * math.pi - math.acos(costheta)
        else:
            theta = math.acos(costheta)
        print(distance)
        print(theta)
        env.setState(distance, 0)
        #theta = math.asin(pose.x/pose.length())
        env.setState(theta, 1)
def setCollusion(data):
    """

    :param data:
    :return:
    """
    env.setCollusion(True)
    #env.setCollusion(False)
def getGoal(data):
    #print("get goal!")
    goal.x = data.pose.position.x
    goal.y = data.pose.position.y
    #print("goal:{} {}".format(goal.x, goal.y))
    msg = GoalID()
    pubStopMovebase.publish(msg)
    receiveGoal = True
    #print("publish goal stop")

def init():
    global pubAction,pubStopMovebase
    rospy.init_node('mainController', anonymous=True)
    pubAction = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    env.setPublish(pubAction)
    pubStopMovebase = rospy.Publisher('/move_base/cancel',GoalID,queue_size = 10)
    subPosition = rospy.Subscriber("/odometry/filtered", Odometry, getPosition)
    subGoal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, getGoal)
    while True:
        env.step(1)
        #movecontrol(v, r)
        rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    try:
        init()
    except rospy.ROSInterruptException as e:
        print(e.args[0])