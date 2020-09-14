#! /usr/bin/env python

import argparse
import rospy
from geometry_msgs.msg import Twist
import sys, select, tty, termios
import thread


def get_velocity(c):
    if c == 'a' or c == 'A':
        return 0, 0.1
    if c == 'd' or c == 'D':
        return 0, -0.1
    if c == 'w' or c == 'W':
        return 0.1, 0
    if c == 'x' or c == 'x':
        return -0.1, 0
    return 0,0

def key_catch(cfg):
    rospy.init_node('keyboard')
    pub = rospy.Publisher(cfg.topic, Twist, queue_size=1)
    rate = rospy.Rate(10)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    vx = 0
    vz = 0
    print('Please input keys, press Ctrl + C to quit')
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            ch = sys.stdin.read(1)
            if ch == 's' or ch == 'S':
                vx = 0
                vz = 0
            else:
                dx, dz = get_velocity(ch)
                vx += dx
                vz += dz
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = vz
        pub.publish(msg)
        print("the linear velocity is {} and the angular velocity is {}              \r".format(vx, vz)),
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    return

if __name__=='__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("topic", help="The topic to control the robot move, e.g. /cmd_vel")
        config = parser.parse_args()
        key_catch(config)
    except rospy.ROSInterruptException:
        pass
