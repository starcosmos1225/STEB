#! /usr/bin/env python
from geometry_msgs.msg import Twist
import numpy as np
import math
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from Vec import Vec2d
class Interact:
    def __init__(self, name, state_n=5, action_n=3, mapsize=100):
        self.name = name
        self.action_n = action_n
        self.state_n = state_n
        self.state = [None]*5
        self.size = mapsize
        self.action_list = [(0.2, -0.2),
                            (0.5, 0),
                            (0.2, 0.2)]
                            #(0, 0)]
        self.velocity = 0
        self.rad = 0
        self.state[2] = self.velocity
        self.state[3] = self.rad
        self.oldDistance = None
        self.oldV = 0
        self.oldR = 0
        self.distanceLimit = 1.0*20
        self.reward = 200
        self.collision = False
        self.initPose = np.mat([[1, 0, 0]])
        self.pose = Vec2d()
        self.goal = Vec2d()
        self.position = Vec2d()
        self.receiveGoal = False
        self.globalMap = np.zeros((self.size * 2, self.size * 2, 1))
        self.globalMap_ = np.zeros((self.size, self.size, 1))
        self.pubAction = rospy.Publisher('/'+name + '/cmd_vel', Twist, queue_size=10)
        self.pubStopMovebase = rospy.Publisher('/'+name + '/move_base/cancel', GoalID, queue_size=10)
        self.pubGoal = rospy.Publisher('/' + name + "/move_base_simple/goal", PoseStamped, queue_size=10)
        self.subPosition = rospy.Subscriber('/'+name + "/odometry/filtered", Odometry, self.getPosition)
        self.subCollision = rospy.Subscriber('/'+name + "/scan", LaserScan, self.getCollision)
        self.subCostmap = rospy.Subscriber('/'+name + "/map", OccupancyGrid, self.costmap2local)
        self.subVel = rospy.Subscriber('/' + name + '/cmd_vel', Twist, self.getVelocity)
        #print("{} finish init".format(name))

    def getState(self):
        state = (self.state[0], self.state[1], self.state[2], self.state[3], self.state[4])
        return np.array(state)

    def clearState(self):
        self.state = [None] * self.state_n
        self.velocity = 0
        self.rad = 0
        self.state[2] = self.velocity
        self.state[3] = self.rad

    def setState(self, state, index):
        #print("get state{} index{}".format(state,index))
        self.state[index] = state

    def step(self, action):
        print("action:{}".format(action))
        self.oldV = self.velocity
        self.oldR = self.rad
        self.oldDistance = self.state[0]
        self.velocity, self.rad = self.action_list[action]
        self.setState(self.velocity, 2)
        self.setState(self.rad, 3)
        msg = Twist()
        msg.linear.x = self.velocity
        msg.angular.z = self.rad
        for i in range(5):
            self.pubAction.publish(msg)
            rospy.sleep(0.2)

    def getVelocity(self, data):
        self.velocity = data.linear.x
        self.rad = data.angular.z
        pass
    def getStatus(self):
        done = self.state[0] < self.distanceLimit
        if not done:
            current_reward = 100 * (self.oldDistance - 0.00001 - self.state[0]) - 0.001 * (
                    math.fabs(self.velocity - self.oldV) + math.fabs(self.rad - self.oldR))
            #print("old:{}".format(self.oldDistance))
            #print("newd:{}".format(self.state[0]))
            #print("vdis:{}".format(math.fabs(self.velocity - self.oldV)))
            #print("rdis:{}".format(math.fabs(self.rad - self.oldR)))
            if self.collision:
                current_reward -= 10000
        else:
            print("reach goal!")
            current_reward = self.reward
        if self.collision:
            print("hit occur!")
            done = True
        self.oldDistance = self.state[0]
        return self.getState(), current_reward, done, {}

    def getAction(self):
        return self.action_n

    def setCollision(self, IsCollision):
        self.collision = IsCollision

    def ready(self):
        for i in range(self.state_n):
            if self.state[i] is None:
                #print("state {} is None".format(i))
                return False
        return True
    def getPosition(self, data):
        self.position.x = data.pose.pose.position.x
        self.position.y = data.pose.pose.position.y
        x, y, z, w = data.pose.pose.orientation.x, data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w
        rotateMat = np.mat([[1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*w*y+2*x*z],
                            [2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x],
                            [2*x*z-2*w*y, 2*w*x+2*y*z, 1-2*x*x-2*y*y]])
        pose_ = self.initPose*rotateMat.transpose()
        self.pose = Vec2d(pose_[0, 0], pose_[0, 1]).norm()
        #print("pose:{} {}".format(self.pose.x, self.pose.y))
        #print("position:{} {}".format(self.position.x, self.position.y))
        if self.receiveGoal:
            d = self.goal-self.position
            distance = d.length()*20
            d = d.norm()
            sintheta = self.pose.cross(d)
            try:
                costheta = min(max(d.dot(self.pose), -1.0), 1.0)
                if sintheta > 0:
                    theta = 2 * math.pi - math.acos(costheta)
                else:
                    theta = math.acos(costheta)
            except ValueError as e:
                print("value error:d:{} {} pose:{} {}".format(d.x, d.y, pose.x, pose.y))
            #print("distance:{}theta:{}".format(distance,theta))
            self.setState(distance, 0)
            if self.oldDistance is None:
                self.oldDistance = distance
            self.setState(theta, 1)

    def sendGoal(self):
        msg = PoseStamped()
        msg.header.frame_id = self.name + '_tf/map'
        msg.pose.position.x = self.goal.x
        msg.pose.position.y = self.goal.y
        msg.pose.position.z = 0
        msg.pose.orientation.x, msg.pose.orientation.y, \
        msg.pose.orientation.z, msg.pose.orientation.w = \
            0, 0, 0, 1
        self.pubGoal.publish(msg)

    def getCollision(self, data):
        #print(data.ranges)
        #print("***********************************************************")
        for r in range(30, len(data.ranges), 30):
            #print(data.ranges[r])
            if 0.3 < data.ranges[r] < 0.6:
                print(data.ranges[r])
                #print("hit!collision:{}".format(data.ranges[r]))
                self.setCollision(True)
                return
        self.setCollision(False)

    def costmap2local(self, data):
        #print(data.header)
        VerticalPose = Vec2d(self.pose.y, -self.pose.x)*data.info.resolution
        HorizonPose = Vec2d(self.pose.x, self.pose.y)*data.info.resolution
        LDPosition = self.position - HorizonPose*0.5*self.size - VerticalPose*0.5*self.size
        currentPosition = Vec2d(LDPosition.x, LDPosition.y)
        # notice x,y is not row and col but the axis,the map's x is car's init pose.x
        # so if image is from top to buttom and left to right from car's vision
        # in the data.data we must read from xright to xleft and yup to ydown
        map_image = np.zeros((self.size, self.size, 1))

        for i in range(self.size):
            for j in range(self.size):
                mapPosition = self.toMapPosition(currentPosition, data.info)
                if mapPosition.x < 0 or mapPosition.x >= data.info.width or \
                    mapPosition.y < 0 or mapPosition.x >= data.info.height:
                    map_image[i, j, 0] = 1.0
                else:
                    map_image[i, j, 0] = self.toColor(data.data[mapPosition.x+mapPosition.y*data.info.width])
                #move to next position
                currentPosition = currentPosition + VerticalPose
            currentPosition = LDPosition + HorizonPose*i
        LDPosition = self.position - HorizonPose * self.size - VerticalPose * self.size
        currentPosition = Vec2d(LDPosition.x, LDPosition.y)
        #self.globalMap = np.zeros((self.size * 2, self.size * 2, 1))
        #print(self.globalMap.shape)
        #print("wtf")
        #t=input("wtf")
        #for i in range(self.size*2):
            #for j in range(self.size*2):
                #mapPosition = self.toMapPosition(currentPosition, data.info)
                #if mapPosition.x < 0 or mapPosition.x >= data.info.width or \
                    #mapPosition.y < 0 or mapPosition.x >= data.info.height:
                    #self.globalMap[i, j, 0] = 1.0
                #else:
                    #self.globalMap[i, j, 0] = self.toColor(data.data[mapPosition.x+mapPosition.y*data.info.width])
                #move to next position
                #currentPosition = currentPosition + VerticalPose
            #currentPosition = LDPosition + HorizonPose*i
        #self.globalMap_ = self.compressMap(self.globalMap, compression=1)
        self.setState(self.compressMap(map_image, compression=5), 4)
        #print("end costmap")
    def toMapPosition(self, currentPosition, dataInfo):
        return Vec2d(int((currentPosition.x-dataInfo.origin.position.x)/dataInfo.resolution),
                 int((currentPosition.y-dataInfo.origin.position.y)/dataInfo.resolution))

    def toColor(self, cost):
        if cost == -1:
            return 0.0
        if cost<10:
            return 0.0
        return 1.0

    def toMapPosition(self, currentPosition, dataInfo):
        return Vec2d(int((currentPosition.x-dataInfo.origin.position.x)/dataInfo.resolution),
                 int((currentPosition.y-dataInfo.origin.position.y)/dataInfo.resolution))

    def getGlobalMap(self):
        return self.globalMap_.copy()

    def compressMap(self, map_image, compression=1):
        if compression == 1:
            return map_image.copy()
        size = map_image.shape[0]
        s = int(size/compression)
        data = np.zeros((s, s, 1))
        for i in range(size):
            for j in range(size):
                x = int(i/compression)
                y = int(j/compression)
                if map_image[i, j, 0] > 1e-6:
                    data[x, y, 0] = 1.0
        return data.copy()
def test():
    rospy.init_node('mainController', anonymous=True)
    i = Interact(name='h1')
    i.receiveGoal=True
    i.goal.x = -8
    i.goal.y = -2
    rospy.spin()
#if __name__=='__main__':
    #test()