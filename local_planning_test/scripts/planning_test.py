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
import math
from nav_msgs.msg import Odometry
h1_odom = None
h2_odom = None
h3_odom = None
tf_buffer = None
tf_listener_1 = None
store = []
predict_no = 50
dynamic_dt = 0.2
radius = 2.0
center = [5,0.0]
initTheta = 3.1415926
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
    a = open('test.txt', 'a')
    a.write("{} {} {} {} {}\n".format(h1_odom.pose.pose.position.x,h1_odom.pose.pose.position.y,h2_odom.pose.pose.position.x,h2_odom.pose.pose.position.y,time))
    a.close()


def computelinearVelocity(dt, vx, vy, position, acc_x, acc_y, time):
    global h2_odom
    h2_obstacles = ObstacleMsg()
    h2_obstacles.header = h2_odom.header
    point = Point32()
    next_vx = max(-1.0, min(acc_x*20*dt + vx,0.0))
    next_vy = max(-1.0, min(acc_y*20*dt + vy,0.0))
    point.x = position[0, 0] + 0.5*(next_vx+vx)*dt
    point.y = position[1, 0] + 0.5*(next_vy+vy)*dt
    position[0, 0] = point.x
    position[1, 0] = point.y
    vx = next_vx
    vy = next_vy
    point.z = time
    h2_obstacles.polygon.points.append(point)
    h2_obstacles.radius = 0.5
    h2_obstacles.velocities = h2_odom.twist
    #print("h2:{} {} {} vx {} vy {} acc_x {} acc_y {}".format(point.x,point.y,point.z,vx,vy,acc_x,acc_y))
    h2_obstacles.orientation = h2_odom.pose.pose.orientation
    return h2_obstacles, vx, vy, position


def computeCircle(dt, angular, theta, time):
    global center, radius, h2_odom
    h2_obstacles = ObstacleMsg()
    h2_obstacles.header = h2_odom.header
    point = Point32()
    next_angular = angular + theta*dt
    newPosition = computePosition(center, radius, next_angular)
    point.x = newPosition[0, 0]
    point.y = newPosition[1, 0]
    point.z = time
    h2_obstacles.polygon.points.append(point)
    h2_obstacles.radius = 0.5
    h2_obstacles.velocities = h2_odom.twist
    #print("h2:{} {} {} ".format(point.x,point.y,point.z))
    h2_obstacles.orientation = h2_odom.pose.pose.orientation
    return h2_obstacles, next_angular


def computeAngle(p1,p2):
    l = ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**0.5
    dx = p1[0]-p2[0]
    dy = p1[1]-p2[1]
    costheta = dx/l
    theta = math.acos(costheta)
    if dy<0:
        theta = 2*math.pi-theta
    return theta

def computePosition(c, r, angular):
    dx = r*math.cos(angular)
    dy = r*math.sin(angular)
    p = np.zeros((2, 1))
    p[0, 0] = c[0]+dx
    p[1, 0] = c[1]+dy
    return p


def run():
    global h2_odom,h3_odom, tf_buffer, tf_listener_1,center
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
    pub_h1_obstacless = rospy.Publisher("/move_base/TebLocalPlannerROS/dynamic_obstacles", ObstacleArrayMsg, queue_size=1)
    #pub_h1_obstacless = rospy.Publisher("/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, queue_size=1)

    rate = rospy.Rate(20)
    count = 0
    h2_vx = 1.0
    theta = 0
    acc = -0.0035
    while not rospy.is_shutdown():
        # try:
        #     (trans, rot) = tf_listener.lookupTransform('/map', '/h2/odom', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue
        # print(type(trans))
        # print(type(rot))

        # print(h2_odom)
        # print(h3_odom)
        if h2_odom is not None:
            msg = ObstacleArrayMsg()
            msg.header = h2_odom.header
            nowTime = rospy.Time.now().secs
            position = np.zeros((2, 1))
            position[0, 0] = h2_odom.pose.pose.position.x
            position[1, 0] = h2_odom.pose.pose.position.y
            vx = h2_odom.twist.twist.linear.x
            vy = h2_odom.twist.twist.linear.y
            acc_x = acc*vx/np.sqrt(vx*vx+vy*vy)
            acc_y = acc*vy/np.sqrt(vx*vx+vy*vy)
            angular = computeAngle(position, center)
            teb_msg = ObstacleArrayMsg()
            teb_msg.header = h2_odom.header
            for i in range(predict_no):
                if i ==0:
                    h2_obstacles = ObstacleMsg()
                    h2_obstacles.header = h2_odom.header
                    point = Point32()
                    point.x = position[0, 0]
                    point.y = position[1, 0]
                    point.z = nowTime
                    #point.z = 0.0
                    h2_obstacles.polygon.points.append(point)
                    h2_obstacles.radius = 0.5
                    h2_obstacles.velocities = h2_odom.twist
                    teb_msg.obstacles.append(h2_obstacles)
                    msg.obstacles.append(h2_obstacles)
                else:
                    h2_obstacles, vx, vy, position = computelinearVelocity(dynamic_dt,vx,vy,position,acc_x,acc_y,nowTime + dynamic_dt*i)
                    #h2_obstacles, angular = computeCircle(dynamic_dt,angular,theta,nowTime + dynamic_dt*i)
                    msg.obstacles.append(h2_obstacles)
            pub_h1_obstacless.publish(msg)
            #pub_h1_obstacless.publish(teb_msg)
        if count < 20:
            # print("begin pub")
            msg = PoseStamped()
            msg.header.seq = count
            msg.header.frame_id = "map"
            msg.pose.position.x = -3
            msg.pose.position.y = 0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            pub_h1_goal.publish(msg)
        # if 40 < count < 190: this seting will hit the robot
        if 20< count:
            msg_h2 = Twist()
            msg_h2.linear.x = min(h2_vx,1.0)
            msg_h2.angular.z = theta
            #if count==150:
            #    acc = -0.01
            #if count==270:
            #    acc = 0
            h2_vx += acc
            if h2_vx <0:
                acc=0
                h2_vx=0
            pub_h2.publish(msg_h2)
        if 20< count < 1800:
            msg_h3 = Twist()
            msg_h3.linear.x = 0.30#use teb set it 0.30,use steb set it 0.45
            msg_h3.angular.z = 0.0
            #pub_h3.publish(msg_h3)
        if count >= 900:
            h2_vx=0
        count += 1
        # save_position()
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

