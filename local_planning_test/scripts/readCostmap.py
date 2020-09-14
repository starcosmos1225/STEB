#! /usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import Odometry
from PIL import Image
import numpy as np
image_array = np.zeros((4000,4000), dtype='uint8')
positionX=0
positionY=0
def getCostMap(data):
    #print(data.header)
    global image_array
    print(data.info)
    #print(type(data.data))
    #image_array = np.zeros((4000, 4000), dtype='uint8')
    c = 0
    for i in range(4000):
        for j in range(4000):
            if data.data[c] == -1:
                image_array[i][j] = 0
            else:
                image_array[i][j] = 255 if data.data[c] > 50 else 0
            c += 1
    for i in range(-2,3):
        for j in range(-2, 3):
            image_array[positionX+i+2000][positionY+j+2000] = 127
    im = Image.fromarray(image_array)
    im = im.convert('L')
    im.save("h.png")
    pass
def getCostMapUpdate(data):
    global image_array
    print(data.x)
    print(data.y)
    print(data.width)
    print(data.height)
    print(len(data.data))
    c=0
    for i in range(data.width):
        for j in range(data.height):
            if data.data[c] == -1:
                image_array[i][j] = 0
            else:
                image_array[i][j] = 255 if data.data[c] > 50 else 0
            c += 1
    for i in range(-2, 3):
        for j in range(-2, 3):
            image_array[positionX+i+2000][positionY+j+2000] = 127
    im = Image.fromarray(image_array)
    im = im.convert('L')
    im.save("h.png")
    pass
def getPosition(data):
    #print("receive position:{} {}".format(data.pose.pose.position.x,data.pose.pose.position.y))
    global positionX,positionY
    positionX = int(data.pose.pose.position.x)
    positionY = int(data.pose.pose.position.y)

def init():
    rospy.init_node('readCostMap', anonymous=True)
    #subCostMap = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, getCostMap)
    subCostMapUpdate = rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, getCostMapUpdate)
    subPosition = rospy.Subscriber("/odometry/filtered", Odometry, getPosition)
    rospy.spin()
if __name__=="__main__":
    try:

        # child.terminate()
        init()
    except rospy.ROSInterruptException as e:
        print(e.args[0])