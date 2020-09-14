#! /usr/bin/env python

import rospy
from collections import deque
from robottest.msg import emergency_msg
from robottest.msg import emergency_env
from robottest.msg import moveCommand
from robottest.msg import keyString
def emergencyController(data,cmdqueue):
    if data.impactOccur==True:
        print("impact occur!")
        pub = rospy.Publisher('emergency_env', emergency_env, queue_size=10)
        msg=emergency_env()
        msg.mainControllerBreak=True
        pub.publish(msg)
        rate = rospy.Rate(1)
        while (len(cmdqueue)>0):
            ch=cmdqueue.popleft()
            pubcmd = rospy.Publisher('cmd_vel', moveCommand, queue_size=10)
            msg = moveCommand()
            if ch == 'w':
                msg.linear.x = -2.0
                msg.angular.z = 0
            elif ch == 's':
                msg.linear.x = 2.0
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
            print("emergency command {}!".format(ch))
            pubcmd.publish(msg)
            rate.sleep()
    else:
        pub = rospy.Publisher('emergency_env', emergency_env, queue_size=10)
        msg = emergency_env()
        msg.mainControllerBreak = False
        pub.publish(msg)
    return
def cmdSave(data,args):
    cmdqueue=args[0]
    cmdlength=args[1]
    print(cmdqueue)
    print(cmdlength)
    if (len(cmdqueue)<cmdlength):
        cmdqueue.append(data.key)
    else:
        cmdqueue.popleft()
        cmdqueue.append(data.key)

def emergencyHandle():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('emergency_node', anonymous=True)
    cmdqueue = deque([])
    cmdlength = 3
    rospy.Subscriber("emergency_alarm", emergency_msg, emergencyController,cmdqueue)
    rospy.Subscriber("cmdcopylist", keyString, cmdSave,(cmdqueue,cmdlength))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        emergencyHandle()
    except:
        pass