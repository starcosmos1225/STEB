#! /usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from Vec import Vec2d
from PIL import Image


class GridMap:
    def __init__(self, name, env, size=200,):
        #rospy.init_node('costmapTransform', anonymous=True)
        subCostmap = rospy.Subscriber(name+"/map", OccupancyGrid, self.costmap2local)
        self.carPose = Vec2d(1, 0)
        self.carPosition = Vec2d(0, 0)
        self.initPose = np.mat([[1, 0, 0]])
        self.map_image = np.zeros((size, size, 1))
        self.size = size
        self.env = env
        #rospy.spin()

    def setPosition(self, position, pose):
        self.carPosition.x, self.carPosition.y = position.x, position.y
        self.carPose.x, self.carPose.y = pose.x, pose.y

    def toColor(self, cost):
        if cost==-1:
            return 0
        if cost<10:
            return 0
        return 255

    def toMapPosition(self, currentPosition, dataInfo):
        return Vec2d(int((currentPosition.x-dataInfo.origin.position.x)/dataInfo.resolution),
                 int((currentPosition.y-dataInfo.origin.position.y)/dataInfo.resolution))

    def costmap2local(self, data):
        #print(data.info)
        VerticalPose = Vec2d(self.carPose.y, -self.carPose.x)*data.info.resolution
        HorizonPose = Vec2d(self.carPose.x, self.carPose.y)*data.info.resolution
        LUPosition = self.carPosition + HorizonPose*0.5*self.size - VerticalPose*0.5*self.size
        currentPosition = Vec2d(LUPosition.x, LUPosition.y)
        # notice x,y is not row and col but the axis,the map's x is car's init pose.x
        # so if image is from top to buttom and left to right from car's vision
        # in the data.data we must read from xright to xleft and yup to ydown
        for i in range(self.size-1, -1, -1):
            for j in range(self.size):
                mapPosition = self.toMapPosition(currentPosition, data.info)
                if mapPosition.x < 0 or mapPosition.x >= data.info.width or \
                    mapPosition.y < 0 or mapPosition.x >= data.info.height:
                    self.map_image[i, j, 0] = 1.0
                else:
                    self.map_image[i, j, 0] = self.toColor(data.data[mapPosition.x+mapPosition.y*data.info.width])
                #move to next position
                currentPosition = currentPosition + VerticalPose
            currentPosition = LUPosition - HorizonPose*i
        self.env.setState(self.getData(compression=5), 4)
    def getData(self,compression=1):
        if compression == 1:
            return self.map_image.copy()
        s = int(self.size/compression)
        data = np.zeros((s, s, 1))
        for i in range(self.size):
            for j in range(self.size):
                x = i/compression
                y = j/compression
                if self.map_image[i, j, 0] == 1.0:
                    data[x, y, 0] = 1.0
        return data.copy()


