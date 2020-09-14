#! /usr/bin/env python

import argparse
import rospy
from geometry_msgs.msg import Twist,Point32
from geometry_msgs.msg import PoseStamped
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
import sys, select, tty, termios
import thread
import tf2_ros
import tf
import tf2_geometry_msgs
import numpy as np
from nav_msgs.msg import Odometry
h1_odom = None
h2_odom = None
h3_odom = None
tf_buffer = None
tf_listener_1 = None
store = []
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
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = vz
        pub.publish(msg)
        print("the linear velocity is {} and the angular velocity is {}              \r".format(vx, vz)),
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    return

def h1_odom_cb(data):
    global h1_odom,tf_buffer, tf_listener_1
    if tf_buffer is None:
        return
    h1_odom = data

def h2_odom_cb(data):
    global h2_odom,tf_buffer, tf_listener_1
    if tf_buffer is None:
        return
    pose_h2 = PoseStamped()
    pose_h2.header = data.header
    pose_h2.pose = data.pose.pose
    data_frame = data.header.frame_id
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    transform = tf_buffer.lookup_transform("odom",
                                           data_frame,  # source frame
                                           rospy.Time(0),  # get the tf at first available time
                                           rospy.Duration(1.5))  # wait for 1 second
    h2_in_map = tf2_geometry_msgs.do_transform_pose(pose_h2, transform)
    h2_odom = data
    h2_odom.header.frame_id = "odom"
    h2_odom.pose.pose = h2_in_map.pose
    tf_listener_1.waitForTransform("odom",
                                           data_frame,  # source frame
                                           rospy.Time(0),  # get the tf at first available time
                                           rospy.Duration(1.5))
    (t,rot) = tf_listener_1.lookupTransform("odom",
                                           data_frame,  # source frame
                                           rospy.Time(0))
    #print("frame:{}".format(data_frame))
    #print(rot)
    #print(type(rot))
    r,p,yaw = tf.transformations.euler_from_quaternion(rot)
    if yaw>3.1415926:
        yaw = yaw - 6.283
    if yaw<-3.1415926:
        yaw = yaw + 6.283
    #print("yaw:{}".format(yaw))
    h2_odom.twist.twist.linear.x = vx*np.cos(yaw)
    h2_odom.twist.twist.linear.y = vx*np.sin(yaw)
    h2_odom.twist.twist.linear.z = 0.0



def h3_odom_cb(data):
    global h3_odom, tf_buffer, tf_listener_1
    if tf_buffer is None:
        return
    data_frame = data.header.frame_id
    pose_h3 = PoseStamped()
    pose_h3.header = data.header
    pose_h3.pose = data.pose.pose
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    #pose_v.pose.position.z = data.twist.twist.linear.z
    transform = tf_buffer.lookup_transform("odom",data_frame,
                                           rospy.Time(0),  # get the tf at first available time
                                           rospy.Duration(1.5))  # wait for 1 second
    h3_in_map = tf2_geometry_msgs.do_transform_pose(pose_h3, transform)
    tf_listener_1.waitForTransform("odom",
                                           data_frame,  # source frame
                                           rospy.Time(0),  # get the tf at first available time
                                           rospy.Duration(1.5))
    (t,rot) = tf_listener_1.lookupTransform("odom",
                                           data_frame,  # source frame
                                           rospy.Time(0))
    # h3_in_map_v = tf2_geometry_msgs.do_transform_pose(pose_v, transform)
    h3_odom = data
    h3_odom.header.frame_id = "odom"
    h3_odom.pose.pose = h3_in_map.pose
    r,p,yaw = tf.transformations.euler_from_quaternion(rot)
    if yaw>3.1415926:
        yaw = yaw - 6.283
    if yaw<-3.1415926:
        yaw = yaw + 6.283
    
    h3_odom.twist.twist.linear.x = vx*np.cos(yaw)
    h3_odom.twist.twist.linear.y = vx*np.sin(yaw)
    h3_odom.twist.twist.linear.z = 0.0

def storage():
    if h1_odom is None or h2_odom is None:
        return
    time = rospy.Time.now()
    a=open('test.txt', 'a')
    a.write("{} {} {} {} {}\n".format(h1_odom.pose.pose.position.x,h1_odom.pose.pose.position.y,h2_odom.pose.pose.position.x,h2_odom.pose.pose.position.y,time))
    a.close()


def run():
    global h2_odom,h3_odom, tf_buffer, tf_listener_1
    rospy.init_node('local_planning_test_script')
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_listener_1 = tf.TransformListener()
    pub_h2 = rospy.Publisher("/h2/cmd_vel", Twist, queue_size=1)
    pub_h3 = rospy.Publisher("/h3/cmd_vel", Twist, queue_size=1)
    pub_h1_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    sub_h1_pose = rospy.Subscriber("/odometry/filtered", Odometry, h1_odom_cb)
    sub_h2_pose = rospy.Subscriber("/h2/odometry/filtered", Odometry, h2_odom_cb)
    sub_h3_pose = rospy.Subscriber("/h3/odometry/filtered", Odometry, h3_odom_cb)
    pub_h1_obstacless = rospy.Publisher("/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, queue_size=1)

    rate = rospy.Rate(20)
    count = 0
    while not rospy.is_shutdown():
        # try:
        #     (trans, rot) = tf_listener.lookupTransform('/map', '/h2/odom', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue
        # print(type(trans))
        # print(type(rot))

        # print(h2_odom)
        # print(h3_odom)
        if h2_odom is not None and h3_odom is not None:
            h2_obstacles = ObstacleMsg()
            h2_obstacles.header = h2_odom.header
            point = Point32()
            point.x = h2_odom.pose.pose.position.x
            point.y = h2_odom.pose.pose.position.y
            point.z = h2_odom.pose.pose.position.z
            h2_obstacles.polygon.points.append(point)
            h2_obstacles.radius = 0.5
            h2_obstacles.velocities = h2_odom.twist
            #print("h2 velocity:{} {}".format(h2_odom.twist.twist.linear.x,h2_odom.twist.twist.linear.y))
            h2_obstacles.orientation = h2_odom.pose.pose.orientation
            h3_obstacles = ObstacleMsg()
            point1 = Point32()
            point1.x = h3_odom.pose.pose.position.x
            point1.y = h3_odom.pose.pose.position.y
            point1.z = h3_odom.pose.pose.position.z
            h3_obstacles.header = h3_odom.header
            h3_obstacles.polygon.points.append(point1)
            h3_obstacles.radius = 0.5
            h3_obstacles.velocities = h3_odom.twist
            h3_obstacles.orientation = h3_odom.pose.pose.orientation
            msg = ObstacleArrayMsg()
            msg.header = h2_obstacles.header
            msg.obstacles.append(h2_obstacles)
            msg.obstacles.append(h3_obstacles)
            pub_h1_obstacless.publish(msg)
        if count < 20:
            #print("begin pub")
            msg = PoseStamped()
            msg.header.seq = count
            msg.header.frame_id = "map"
            msg.pose.position.x = -6
            msg.pose.position.y = 0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            pub_h1_goal.publish(msg)
        # if 40 < count < 190: this seting will hit the robot
        if 20< count< 380:
            msg_h2 = Twist()
            msg_h2.linear.x = 1.0
            msg_h2.angular.z = 0.0
            pub_h2.publish(msg_h2)
        if 20< count < 1800:
            msg_h3 = Twist()
            msg_h3.linear.x = 0.30#use teb set it 0.30,use steb set it 0.45
            msg_h3.angular.z = 0.0
            #pub_h3.publish(msg_h3)
        if count >= 2800:
            break
        count += 1
        storage()
        rate.sleep()
    return

if __name__=='__main__':
    try:
        #parser = argparse.ArgumentParser()
        #parser.add_argument("topic", help="The topic to control the robot move, e.g. /cmd_vel")
        #config = parser.parse_args()
        #key_catch(config)
        run()
    except rospy.ROSInterruptException:
        pass

