#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ContactsState
import matplotlib.pyplot as plt
from numpy import *
first=True
def sensor_laser(data):
    import matplotlib.pyplot as plt
    global first
    global num
    ycord = []
    length = len(data.ranges)
    #it is a wrong algorithm with the distance!
    xcord = []
    core = [0, 0]
    distance = 0.0
    for i in range(length):
        if (float('inf') == data.ranges[i]):
            distance = data.range_max
        else:
            distance = data.ranges[i]
        rad=data.angle_max-i*data.angle_increment
        xcord.append(sin(rad)*distance)
        ycord.append(cos(rad)*distance)
    if first:
        plt.ion()
        plt.figure(1)
        first=False
    plt.clf()
    plt.plot(xcord,ycord)
 #   plt.ioff()
    plt.draw()
    return
def sensor_contact(data):
 #   print("states: {}".format(data.states))
 #   print("the data length is :{}".format(len(data.states)))
    return
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('display', anonymous=True)
    rospy.Subscriber("/robottest/scan", LaserScan,sensor_laser)
    rospy.Subscriber("/robottest/bumper", ContactsState, sensor_contact)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
        plt.close()
    except:
        plt.close()
        pass

