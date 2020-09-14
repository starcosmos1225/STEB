import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt


mpl.rcParams['legend.fontsize'] = 10
fig = plt.figure()


ax = fig.gca(projection='3d')
ax.legend()


def drawStatic(line):
    theta = np.ones(100)
    z = np.linspace(0, 8, 100) / 4
    for L in line:
        x = L[0]*theta
        y = L[1]*theta
        ax.plot(x, y, z, color='orange', label="static obstacle")
def drawCurve(data,c='red'):
    x = data[:,0]
    y = data[:,1]
    z = data[:,2]
    ax.plot(x,y,z,color=c,label="static obstacle")
    # x_l = []
    # y_l = []
    # z_l = []
    # x_d = []
    # y_d = []
    # z_d = []
    # for i in range(20):
    #     x_l.append(x[i*5])
    #     y_l.append(y[i*5])
    #     z_l.append(z[i*5])
    #     x_d.append(x[i*5+1]-x[i*5])
    #     y_d.append(y[i * 5 + 1] - y[i * 5])
    #     z_d.append(z[i * 5 + 1] - z[i * 5])
    # ax.quiver(x_l,y_l, z_l, x_d, y_d, z_d, length=5, color='red')

def drawDy1():
    y = np.linspace(1, -2, 100)
    x = -2/3*y+2/3
    z = np.zeros(100)
    for i in range(1,100):
        z[i] = 0.0002*i*i+0.01
    ax.plot(x, y, z, color='blue',label="static obstacle")
    x_l = []
    y_l = []
    z_l = []
    x_d = []
    y_d = []
    z_d = []
    for i in range(20):
        if i>0:
            x_l.append(x[i * 5])
            y_l.append(y[i * 5])
            z_l.append(z[i * 5])
            x_d.append(x[i * 5 + 1] - x[i * 5])
            y_d.append(y[i * 5 + 1] - y[i * 5])
            z_d.append(z[i * 5 + 1] - z[i * 5])
    ax.quiver(x_l, y_l, z_l, x_d, y_d, z_d, length=5,  color='blue')
    ax.plot([0],[-2],[0],marker='o',color='orange')
    ax.text(0,-2,0,'start')
    ax.plot([3], [3], [1.5], marker='o', color='orange')
    ax.text(3, 3, 1.5,'end')
def drawDy2():
    x = np.linspace(1, 3, 100)
    y = -2*x+5
    z = np.zeros(100)
    for i in range(1,100):
        z[i] = 0.15*np.sqrt(i)
    ax.plot(x, y, z, color='blue',label="static obstacle")
    x_l = []
    y_l = []
    z_l = []
    x_d = []
    y_d = []
    z_d = []
    for i in range(20):
        if i>0:
            x_l.append(x[i * 5])
            y_l.append(y[i * 5])
            z_l.append(z[i * 5])
            x_d.append(x[i * 5 + 1] - x[i * 5])
            y_d.append(y[i * 5 + 1] - y[i * 5])
            z_d.append(z[i * 5 + 1] - z[i * 5])
    ax.quiver(x_l, y_l, z_l, x_d, y_d, z_d, length=5, color='blue')


def drawFig1():
    staticObstacle = [[2.2, 2]]
    drawStatic(staticObstacle)
    drawDy1()
    drawCurve()

    drawDy2()
    drawStatic([[0.5, -1.2]])

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('t')
    #ax.set_xticks([])
    #ax.set_yticks([])
    #ax.set_zticks([])
    plt.show()

def drawline(pointA, pointB,c='orange'):
    z = np.linspace(pointA[2], pointB[2], 100)
    x = np.linspace(pointA[0], pointB[0], 100)
    y = np.linspace(pointA[1], pointB[1], 100)
    ax.plot(x, y, z, color=c)
START =0.5
CROSS = 1
END =1.05
def drawSqrt(pointA,pointB,c='red'):
    x = np.linspace(pointA[0], pointB[0],100)
    y = np.linspace(pointA[1], pointB[1],100)
    z = np.sqrt(np.sqrt(np.square(x)+np.square(y)))
    ax.plot(x, y, z, color=c)
def drawQuad(pointA,pointB):
    x = np.linspace(pointA[0], pointB[0],100)
    y = np.linspace(pointA[1], pointB[1],100)
    z = np.square(x)+np.square(y)
    ax.plot(x, y, z, color='red')

def drawText(point,text,bias=0):
    ax.text(point[0],point[1],point[2]+bias, text)
def drawDot(point,c='red'):
    ax.plot([point[0]],[point[1]],[point[2]],marker='o',color=c)
def drawArrow(point1,point2,c='black',l='--'):
    direction = [point2[0]-point1[0],point2[1]-point1[1],point2[2]-point1[2]]
    ax.quiver([point1[0]], [point1[1]], [point1[2]], [direction[0]], [direction[1]], [direction[2]], length=1, color=c,
              linestyle=l)
locationS = np.array([START,START,START])
locationA = np.array([CROSS*(0.5**0.5),CROSS*(0.5**0.5),CROSS])
locationB = np.array([END,END,(2**0.25)*np.sqrt(END)])
locationC = np.array([END,END,2*(END**2)])

def drawFig3_1():
    drawSqrt(locationA, locationB)
    drawline([0.6, 1, 1.15 - (0.5 ** 0.5) * 0.2], [1.0, 0.6, 1.15 + (0.5 ** 0.5) * 0.2], c='blue')
    drawline(locationS, locationA)
    drawQuad(locationA, locationC)
    drawText(locationA, 'A',bias=0.05)
    drawText((locationB-locationA)*0.25+locationA, 'C',bias=-0.1)
    drawText((locationC-locationA)*0.25+locationA, 'D',bias=0.05)
    drawDot(locationA)
    drawDot((locationB - locationA) * 0.25 + locationA)
    drawDot((locationC - locationA) * 0.25 + locationA)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('t')
    # ax.set_xticks([])
    # ax.set_yticks([])
    # ax.set_zticks([])
    plt.show()

def drawFig3_2():
    drawline([0.93, 0.0, 0.7], [0.9, 0.0, 1.0], c='slateblue')
    drawline([0.625, 0.0, 0.78], [1.0, 0.0, 0.88], c='navy')
    drawDot([0.84, 0, 0.8], c='darkorange')
    drawDot([0.8, 0, 0.85], c='darkorange')
    drawArrow([0.84, 0, 0.8], [0.72, 0, 0.8], c='dimgray')
    drawArrow([0.84, 0, 0.8], [0.92, 0.0, 0.8], c='dimgray')
    drawArrow([0.84, 0, 0.8], [0.918, 0.0, 0.82], c='crimson', l='-')
    drawArrow([0.84, 0, 0.8], [0.83, 0.0, 0.832], c='crimson', l='-')
    drawArrow([0.84, 0, 0.8], [0.85, 0.0, 0.768], c='deeppink', l='-')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('t')
    ax.set_xlim(0.6,1)
    ax.set_ylim(0, 0.4)
    ax.set_zlim(0.6, 1)
    #ax.set_xticks([])
    #ax.set_yticks([])
    #ax.set_zticks([])
    plt.show()

def drawFig_test():
    global fig
    fr = open("fig.txt")
    data = np.zeros((30,3))
    points = np.zeros((300,3))
    meetpoint=False
    meetedge=False
    index=0
    edges=[]
    length= 0
    first=True
    for line in fr.readlines():
        #print(line)
        if line=="END":
            break
        if line=="plan point:\n":
            meetpoint=True
            meetedge = False
            index=0
            continue
        elif line=="Edge\n":
            meetedge=True
            meetpoint=False
            index=0
            continue
        elif line=="dynamic obstacles\n":
            meetedge=False
            meetpoint=False
            length=index
            index=0
            if first:
                first=False
            else:
                plt.cla()
                ax.set_xlim(0,4)
                ax.set_ylim(-9,9)
                ax.set_zlim(140, 180)
                p = np.zeros((length,3))
                for i in range(length):
                    p[i,:] = points[i,:]
                drawCurve(data,c='blue')
                drawCurve(p)
                for e in edges:
                    if e[0]<30:
                        drawline(data[e[0],:],p[e[1],:])
                
                plt.draw()
                plt.pause(0.5)
                data = np.zeros((30,3))
                points = np.zeros((300,3))
                edges=[]
                
                

            continue
        l = line.strip().split()
        if meetedge:
            edges.append([eval(l[0]),eval(l[1])])
            continue
        for i in range(3):
            if not meetpoint:
                data[index,i] = eval(l[i])
            else:
                points[index,i] = eval(l[i])
        index+=1
    
    fr.close()

if __name__=='__main__':
    drawFig_test()
