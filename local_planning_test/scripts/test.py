#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
def getPosition(data):
    print("get data")
    while True:
        a = 1
def init():
    rospy.init_node('test', anonymous=True)
    pubAction = rospy.Publisher('/test/cmd_vel', Twist, queue_size=10)
    subPosition = rospy.Subscriber("/h1/odometry/filtered", Odometry, getPosition)
    while True:
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        pubAction.publish(msg)
        print("publish finish")
        rospy.sleep(0.5)
    rospy.spin()

if __name__=='__main__':
    init()