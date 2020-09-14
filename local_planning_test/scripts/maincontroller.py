#! /usr/bin/env python

import rospy
from robottest.msg import emergency_env
from robottest.msg import moveCommand
from robottest.msg import keyString
IsBreak=False
def mainControllBreak(data):
    global  IsBreak
    if data.mainControllerBreak==True:
        IsBreak=True
    else :IsBreak=False
def movecontrol(data):
    print(IsBreak)
    if IsBreak:return
    pub = rospy.Publisher('cmd_vel', moveCommand, queue_size=10)
    pubToEmergency = rospy.Publisher('cmdcopylist', keyString, queue_size=10)
    ch=data.key
    msg=moveCommand()
    if ch == 'w':
        msg.linear.x = 2.0
        msg.angular.z = 0
    elif ch == 's':
        msg.linear.x = -2.0
        msg.angular.z = 0
    elif ch == 'a':
        msg.linear.x = 0
        msg.angular.z = 2
    elif ch == 'd':
        msg.linear.x = 0
        msg.angular.z = -2
    else:
        msg.linear.x = 0
        msg.angular.z = 0
    pub.publish(msg)
    pubToEmergency.publish(data)
    return
def talker():
    rospy.init_node('mainController',anonymous=True)
    rospy.Subscriber("emergency_env", emergency_env, mainControllBreak)
    rospy.Subscriber("keyToMainController", keyString, movecontrol)
    rospy.spin()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass