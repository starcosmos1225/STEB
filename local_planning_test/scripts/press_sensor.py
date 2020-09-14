#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ContactsState
from robottest.msg import impact_msg
def sensor_contact(data,publishHandle):
 #   print("states: {}".format(data.states))
    if (len(data.states)>=20):
        msg=impact_msg()
        msg.impactOccur=True
        publishHandle.publish(msg)
    else:
        msg = impact_msg()
        msg.impactOccur = False
        publishHandle.publish(msg)
    return

def press_detect():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('press_sensor', anonymous=True)
    pub = rospy.Publisher('emergency_alarm', impact_msg, queue_size=10)
    rospy.Subscriber("/robottest/bumper", ContactsState, sensor_contact,pub)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        press_detect()
    except:
        pass