import random
import os
import sys
def genRandomWorld(fileName, worlds):
    locations = []
    fr = open(fileName, 'w')
    path = os.path.split(os.path.realpath(__file__))[0]
    headfile = path + "/head.txt"
    tailfile = path + "/tail.txt"
    cubefile = path + "/cube.txt"
    goalfile = path + "/goal.txt"
    hf = open(headfile, "r")
    for line in hf.readlines():
        fr.write(line)
    hf.close()
    for worldInfo in worlds:
        cube_number, W, H, offsetX, offsetY = worldInfo
        locationList = random.sample(range(0, int(W*H/4)), cube_number+2)
        cf = open(cubefile, "r")
        lines = cf.readlines()
        for i in range(cube_number):
            x = locationList[i] / int(0.5 * W)
            y = locationList[i] % int(0.5 * W)
            writeCube(fr, lines, i + offsetX + offsetY, x * 2 + offsetX, y * 2 + offsetY)
        cf.close()
        locationInfo = [locationList[cube_number] / int(0.5 * W) * 2+offsetX,
                        locationList[cube_number] % int(0.5 * W) * 2+offsetY,
                        locationList[cube_number+1] / int(0.5 * W) * 2+offsetX,
                        locationList[cube_number+1] % int(0.5 * W) * 2+offsetY]
        locations.append(locationInfo)
    gf = open(goalfile, "r")
    for line in gf.readlines():
        fr.write(line)
    fr.write("      <pose frame=''>"+str(locationList[cube_number+1] / int(0.5 * W) * 2+offsetX) +
             " "+str(locationList[cube_number+1] % int(0.5 * W) * 2+offsetY)+" 0 0 -0 0</pose>")
    fr.write("    </model>")
    tf = open(tailfile, "r")
    for line in tf.readlines():
        fr.write(line)
    fr.close()
    return locations#locations[i]:positionX,positionY,goalX,goalY
def writeCube(outFile,cubeFileLines,id,x,y):
    global W, H
    outFile.write("    <model name='unit_box_"+str(id)+"'>\n")
    outFile.write("      <pose>" + str(x) + " " + str(y) + " 0 0 0 0</pose>\n")
    for line in cubeFileLines:
        outFile.write(line)
    return
def genEmptyWorld(fileName,W=20, H=20):
    fr = open(fileName, 'w')
    path = os.path.split(os.path.realpath(__file__))[0]
    emptyfile = path + "/empty.txt"
    ef = open(emptyfile, "r")
    carX = 0.0
    carY = 0.0
    goalX = 0.0
    goalY = 0.0
    #dist = (goalX-carX)**2+(goalY-carY)**2
    while (goalX-carX)**2+(goalY-carY)**2 < 20:
        locationList = random.sample(range(0, W*H), 2)
        carX = locationList[0] / W
        carY = locationList[0] % W
        goalX = locationList[1] / W
        goalY = locationList[1] % W
    for line in ef.readlines():
        fr.write(line)
    ef.close()
    fr.close()
    return carX, carY, goalX, goalY

