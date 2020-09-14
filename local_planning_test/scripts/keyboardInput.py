#! /usr/bin/env python

import rospy
from robottest.msg import keyString
import  sys
import  tty, termios
def talker():
    pub=rospy.Publisher('keyToMainController',keyString,queue_size=10)
    rospy.init_node('keyController',anonymous=True)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        # temporary add to skip the python.py
        # break
        fd = sys.stdin.fileno()  # standard input file descriptors
        oldSetting = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)#set the fd file descriptor mode to raw
            ch=sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd,termios.TCSADRAIN,oldSetting)
        msg = keyString()
        msg.key=ch
        pub.publish(msg)
        if ch == 'q': break;
        rate.sleep()
    return

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
