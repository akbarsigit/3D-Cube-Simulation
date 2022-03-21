import sys, math
from graphics import *
from time import sleep
import numpy as np
from math import atan, pi ,sin, cos, sqrt
import math

class Point3D:
    def __init__(self, x = 0, y = 0, z = 0):
        self.x, self.y, self.z = float(x), float(y), float(z)
 
    def rotateX(self, angle):
        """ Rotates the point around the X axis by the given angle in degrees. """
        rad = angle * math.pi / 180
        a = np.sin(rad)
        b = np.cos(rad)
        t = np.array([[1,0,0,0],
                      [0,b,-a,0],
                      [0,a,b,0], 
                      [0,0,0,1]])
        temp = np.dot(t,[self.x,self.y,self.z,1])
        return Point3D(temp[0],temp[1],temp[2])
 
    def rotateY(self, angle):
        """ Rotates the point around the Y axis by the given angle in degrees. """
        rad = angle * math.pi / 180
        a = np.sin(rad)
        b = np.cos(rad)
        t = np.array([[b,0,a,0],
                      [0,1,0,0],
                      [-a,0,b,0],
                      [0,0,0,1]])
        temp = np.dot(t,[self.x,self.y,self.z,1])
        return Point3D(temp[0],temp[1],temp[2])
 
    def rotateZ(self, angle):
        """ Rotates the point around the Z axis by the given angle in degrees. """
        rad = angle * math.pi / 180
        a = np.sin(rad)
        b = np.cos(rad)
        t = np.array([[b,-a,0,0],
                      [a,b,0,0],
                      [0,0,1,0],
                      [0,0,0,1]])
        temp = np.dot(t,[self.x,self.y,self.z,1])
        return Point3D(temp[0],temp[1],temp[2])
 
    def translation(self, x, y, z):
        t = np.array([[1,0,0,x],
                      [0,1,0,y],
                      [0,0,1,z],
                      [0,0,0,1]])
        temp = np.dot(t,[self.x,self.y,self.z,1])
        return Point3D(temp[0],temp[1],temp[2])

    def scaling(self, x, y, z):
        t = np.array([[x,0,0,0],
                      [0,y,0,0],
                      [0,0,z,0],
                      [0,0,0,1]])
        temp = np.dot(t,[self.x,self.y,self.z,1])
        return Point3D(temp[0],temp[1],temp[2])

#point1 and 2 are lists [x1,y1,z1] [x2,y2,z2]
    def RotateAtArbitraryAxis(self, angle, point1, point2):
        xVect = point2[0] - point1[0]
        yVect = point2[1] - point1[1]
        zVect = point2[2] - point1[2]
        beta, miu = 0,0
        if zVect == 0:
            if xVect > 0: 
                beta = 90
            else:
                beta = 270
        else:
            beta = atan(xVect/ zVect) * 180 / pi
        if xVect **2 + zVect**2 == 0:
            if yVect > 0:
                miu = 90
            else:
                miu = 270
        else:
            miu = atan(yVect / math.sqrt(xVect **2 + zVect**2)) * 180 / pi
        x = 0 - point1[0] 
        y = 0 - point1[1] 
        z = 0 - point1[2]
        t1 = np.array([[1,0,0,x],
                      [0,1,0,y],
                      [0,0,1,z],
                      [0,0,0,1]])
        a = np.sin(-beta)
        b = np.cos(-beta)
        rotY1 = np.array([[b,0,a,0],
                      [0,1,0,0],
                      [-a,0,b,0],
                      [0,0,0,1]])
        a = np.sin(miu)
        b = np.cos(miu)
        rotX1 = np.array([[1,0,0,0],
                      [0,b,-a,0],
                      [0,a,b,0], 
                      [0,0,0,1]])
        a = np.sin(angle)
        b = np.cos(angle)
        rotZ = np.array([[b,-a,0,0],
                      [a,b,0,0],
                      [0,0,1,0],
                      [0,0,0,1]])
        a = np.sin(-miu)
        b = np.cos(-miu)
        rotX2 = np.array([[1,0,0,0],
                      [0,b,-a,0],
                      [0,a,b,0], 
                      [0,0,0,1]])
        a = np.sin(beta)
        b = np.cos(beta)
        rotY2 = np.array([[b,0,a,0],
                      [0,1,0,0],
                      [-a,0,b,0],
                      [0,0,0,1]])
        x = point1[0] - 0
        y = point1[1] - 0
        z = point1[2] - 0
        t2 = np.array([[1,0,0,x],
                      [0,1,0,y],
                      [0,0,1,z],
                      [0,0,0,1]])

        temp = np.dot(t2,np.dot(rotY2,np.dot(rotX2,np.dot(rotZ,np.dot(rotX1, np.dot(rotY1,np.dot(t1,[self.x,self.y,self.z,1])))))))

        return Point3D(temp[0],temp[1],temp[2])

    def project(self, win_width, win_height, fov, viewer_distance):
        """ Transforms this 3D point to 2D using a perspective projection. """
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, 1)

class Simulation:
    def __init__(self, win_width = 640, win_height = 480):
        self.vertices = [
            Point3D(-1,1,-1),
            Point3D(1,1,-1),
            Point3D(1,-1,-1),
            Point3D(-1,-1,-1),
            Point3D(-1,1,1),
            Point3D(1,1,1),
            Point3D(1,-1,1),
            Point3D(-1,-1,1)
        ]

        # Define the vertices that compose each of the 6 faces. These numbers are
        # indices to the vertices list defined above.
        self.faces = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]

        self.angleX, self.angleY, self.angleZ = 0, 0, 0
        self.posTransX, self.posTransY, self.posTransZ = 0, 0, 0
        self.posScaleX, self.posScaleY, self.posScaleZ = 1, 1, 1
        self.ArbitratyRotAngle = 0

        self.zzyzx=0
        self.l1 = [[],[],[],[],[],[]]
        self.l2 = [[],[],[],[],[],[]]
        self.l3 = [[],[],[],[],[],[]]
        self.l4 = [[],[],[],[],[],[]] 
        

    def transform(self, w, command, angle=10, x=1, y=1, z=1, point1=[0,0,0], point2=[0,0,0]):
            
            rotAnimX, rotAnimY, rotAnimZ = 0, 0, 0
            posAnimX, posAnimY, posAnimZ = 0, 0, 0
            rotAnimArby = 0
           
            if(command == 'rotX'):
                if(angle < 0):
                    rotAnimX = -1
                else:
                    rotAnimX = 1

            elif(command == 'rotY'):
                if(angle < 0):
                    rotAnimY = -1
                else:
                    rotAnimY = 1

            elif(command == 'rotZ'):
                if(angle < 0):
                    rotAnimZ = -1
                else:
                    rotAnimZ = 1

            elif(command == 'trans'):
                posAnimX, posAnimY, posAnimZ = x/angle, y/angle, z/angle
            
            elif(command == 'scale'):
                self.posScaleX *= x 
                self.posScaleY *= y
                self.posScaleZ *= z

            elif(command == 'arbitraryRotation'):
                self.ArbitratyRotAngle = angle
                # rotAnimArby = 1

        for limit in range(abs(angle)):
            sleep(0.005)
            
            #ANMATION
            self.angleX += rotAnimX
            self.angleY += rotAnimY
            self.angleZ += rotAnimZ

            self.posTransX += posAnimX
            self.posTransY += posAnimY
            self.posTransZ += posAnimZ

            # self.ArbitratyRotAngle += rotAnimArby

            # Will hold transformed vertices.
            t = []
            gx = 0

            for v in self.vertices:

                r = v.rotateX(self.angleX).rotateY(self.angleY).rotateZ(self.angleZ).translation(self.posTransX, self.posTransY, self.posTransZ).scaling(self.posScaleX, self.posScaleY, self.posScaleZ).RotateAtArbitraryAxis(self.ArbitratyRotAngle, point1, point2)
                
                # Transform the point from 3D to 2D
                p = r.project(640, 480, 400, 4)
                # Put the point in the list of transformed vertices
                t.append(p)

            if(self.zzyzx == 1):
                for i in range(6):
                    self.l1[i].undraw()
                    self.l2[i].undraw()
                    self.l3[i].undraw()
                    self.l4[i].undraw()
            else:
                self.zzyzx = 1

            for f in self.faces:
                self.l1[gx]=Line(Point(t[f[0]].x, t[f[0]].y), Point(t[f[1]].x, t[f[1]].y))
                self.l2[gx]=Line(Point(t[f[1]].x, t[f[1]].y), Point(t[f[2]].x, t[f[2]].y))
                self.l3[gx]=Line(Point(t[f[2]].x, t[f[2]].y), Point(t[f[3]].x, t[f[3]].y))
                self.l4[gx]=Line(Point(t[f[3]].x, t[f[3]].y), Point(t[f[0]].x, t[f[0]].y))
                self.l1[gx].setFill('white')
                self.l2[gx].setFill('white')
                self.l3[gx].setFill('white')
                self.l4[gx].setFill('white')
                self.l1[gx].draw(w)
                self.l2[gx].draw(w)
                self.l3[gx].draw(w)
                self.l4[gx].draw(w)
                gx+=1

        
    def run(self):
        """ Main Loop """
        w=GraphWin("3D Wireframe cube Simulation", 640, 480)
        w.setBackground('black')
        Simulation.transform(self, w, "rotX", angle=30)
        Simulation.transform(self, w, 'trans', x=1, y=0, z=0)
        Simulation.transform(self, w, 'rotY', angle=30)
        Simulation.transform(self, w, 'rotZ', angle=30)
        # Simulation.transform(self, w, 'scale', angle=1, x=1.3, y=1, z=1)
        Simulation.transform(self, w, 'arbitraryRotation', angle=30, point1=[1,1,1], point2=[2,2,2])
        input()
               
if __name__ == "__main__":
    Simulation().run()
