"""" 
Tugas TVG Kelompok 17

MARSELLIUS              20/456372/TK/50502
RANGGA AULIA RAHMAN     20/456849/TK/50673
MUHAMAD THORIQ AHNAF    20/460553/TK/51142
AKBAR SIGIT PUTRA       20/463590/TK/51582
RYAN KUSNADI            20/463613/TK/51605

"""

import sys, math
from graphics import *
from time import sleep
import numpy as np
from math import atan, pi ,sin, cos, sqrt
import math

class Point3D:
    #Inisialisasi untuk titik yang akan digunakan vertices
    def __init__(self, x = 0, y = 0, z = 0):
        self.x, self.y, self.z = float(x), float(y), float(z)
  
    #Inisialisasi untuk fungsi rotasi thdp sumbu X
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

    #Inisialisasi untuk fungsi rotasi thdp sumbu Y
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

    #Inisialisasi untuk fungsi rotasi thdp sumbu Z
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

    #Inisialisasi fungsi translasi
    def translation(self, x, y, z):
        t = np.array([[1,0,0,x],
                      [0,1,0,y],
                      [0,0,1,z],
                      [0,0,0,1]])
        temp = np.dot(t,[self.x,self.y,self.z,1])
        return Point3D(temp[0],temp[1],temp[2])

    #Inisialisasi untuk scaling
    def scaling(self, x, y, z):
        t = np.array([[x,0,0,0],
                      [0,y,0,0],
                      [0,0,z,0],
                      [0,0,0,1]])
        temp = np.dot(t,[self.x,self.y,self.z,1])
        return Point3D(temp[0],temp[1],temp[2])
    
    #Inisialisasi Shear (Hanya XY)
    def Shear(self, Shx,Shy):
        t = np.array([
            [1, 0, Shx, 0], 
            [0, 1, Shy, 0],
            [0, 0, 1, 0],
            [0,0,0,1]
            ])
        temp = np.dot(t,[self.x, self.y, self.z, 1])
        return Point3D(temp[0], temp[1], temp[2])

    #point1 dan 2 adalah lists [x1,y1,z1] [x2,y2,z2]
    #Rotasi arbitrary
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
    
    #Proyeksi menjadi 3 dimensi
    def project(self, win_width, win_height, fov, viewer_distance):
        """ Transforms this 3D point to 2D using a perspective projection. """
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, 1)

class Simulation:
    Defaultvertices = [
            Point3D(-2,1,-1),
            Point3D(2,1,-1),
            Point3D(2,-1,-1),
            Point3D(-2,-1,-1),
            Point3D(-2,1,1),
            Point3D(2,1,1),
            Point3D(2,-1,1),
            Point3D(-2,-1,1)
        ]
    def __init__(self, win_width = 800, win_height = 800, tvertices = Defaultvertices):
        self.vertices = tvertices
        

        # Definisikan faces untuk vertices (titik). Di sini maksudnya adalah vertices mana yang membentuk face mana
        # e.g 0,1,2,3 adalah vertices yang membentuk sisi 
        self.faces = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]

        #Definisikan variabel untuk placeholder ketika animasi dilakukan
        self.angleX, self.angleY, self.angleZ = 0, 0, 0
        self.posTransX, self.posTransY, self.posTransZ = 0, 0, 0
        self.posScaleX, self.posScaleY, self.posScaleZ = 1, 1, 1
        self.ArbitratyRotAngle = 0
        self.beta = atan(2/2) * 180 / pi
        self.miu = atan (2 / sqrt(2 **2 + 2**2)) * 180 / pi

        #Placeholder ketika looping
        self.zzyzx=0
        self.l1 = [[],[],[],[],[],[]]
        self.l2 = [[],[],[],[],[],[]]
        self.l3 = [[],[],[],[],[],[]]
        self.l4 = [[],[],[],[],[],[]] 
        

    def transform(self, w, command, angle=10, x=1, y=1, z=1, point1=[0,0,0], point2=[0,0,0], Shx =0 ,Shy =0):

        for limit in range(abs(angle)):
            sleep(0.005)
            
            rotAnimX, rotAnimY, rotAnimZ = 0, 0, 0
            posAnimX, posAnimY, posAnimZ = 0, 0, 0
            self.posShearX, self.posShearY = 0,0

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
            
            elif(command == 'Shear'):
                self.posShearX *= Shx
                self.posShearY *= Shy

            #elif(command == 'arbitraryRotation'):
                #if(angle>0): 
                    #rotAnimArby = -1
               # else:
                    #rotAnimArby=1
                # self.ArbitratyRotAngle += angle
                # rotAnimArby = 1

            #Animasikan
            self.angleX += rotAnimX
            self.angleY += rotAnimY
            self.angleZ += rotAnimZ

            self.posTransX += posAnimX
            self.posTransY += posAnimY
            self.posTransZ += posAnimZ

            self.ArbitratyRotAngle += rotAnimArby

            # Will hold transformed vertices.
            t = []
            gx = 0
            
            self.Defaultvertices = t

            for v in self.vertices:
                #Aplikasikan transformasi ke variabel yang diberikan
                r = v.rotateX(self.angleX).rotateY(self.angleY).rotateZ(self.angleZ).translation(self.posTransX, self.posTransY, self.posTransZ).scaling(self.posScaleX, self.posScaleY, self.posScaleZ).Shear(self.posShearX, self.posShearY).RotateAtArbitraryAxis(self.ArbitratyRotAngle, point1, point2)
                
                # Transformasi titik dari 3 dimensi ke 2 dimensi
                p = r.project(800, 800, 400, 6)
                # Tambahkan ke daftar titik yang di transformasikan
                t.append(p)

            #Hapus line setelah ada perubahan posisi
            if(self.zzyzx == 1):
                for i in range(6):
                    self.l1[i].undraw()
                    self.l2[i].undraw()
                    self.l3[i].undraw()
                    self.l4[i].undraw()
            else:
                self.zzyzx = 1

            #Menggambarkan faces yang didapatkan dari transformasi
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
        w=GraphWin("Transformasi 3 Dimensi Kelompok 17", 800, 800)
        w.setBackground('black')


        msg = """ 
        Select Command:
        1. Define Block 
        2. Translation
        3. Scale
        4. Shear
        5. Rotation on X axis
        6. Rotation on Y axis
        7. Rotation on Z axis
        8. Rotation on Arbitrary axis
        9. Exit
        """
        current_pos =[]
        #Loop untuk animasi
        #Pilih command untuk menampilkan transformasi yang diinginkan
        while True:
            print(msg)
            command = int(input("Enter Command: "))
            if (command == 1):
                Simulation.Defaultvertices = []
                for i in range(8):
                    print("Enter point no ", i+1 , "(example: 1 -1 1):")
                    temp = input().split()
                    current_pos.append(Point3D(int(temp[0]),int(temp[1]),int(temp[2])))
                    Simulation.Defaultvertices = current_pos
            elif (command == 2):
                print("Enter the amount of translation on x y z axis (example: 1 -1 1 ) :")
                input1 = input().split()
                Simulation.transform(self, w, 'trans', x=float(input1[0]), y=float(input1[1]), z=float(input1[2]))
            elif (command == 3):
                print("Enter the scaling factor on x y z axis (example: 1 -1 1 ) :")
                input1 = input().split()
                Simulation.transform(self, w, 'scale', angle=1,x=float(input1[0]), y=float(input1[1]), z=float(input1[2]))
            elif (command == 4):
                print("Enter the shearing factor on x y axis (example: 1 -1) :")
                input1 = input().split()
                Simulation.transform(self,w, 'Shear', Shx = float(input1[0]), Shy =float(input1[1]))
            elif (command == 5):
                print("Enter the rotation angle (in degree):")
                input1 = int(input())
                Simulation.transform(self, w, "rotX", angle=(input1))
            elif (command == 6):
                print("Enter the rotation angle (in degree):")
                input1 = int(input())
                Simulation.transform(self, w, "rotY", angle=(input1))
            elif (command == 7):
                print("Enter the rotation angle (in degree):")
                input1 = int(input())
                Simulation.transform(self, w, "rotZ", angle=(input1))
            elif (command == 8):
                print("Enter the x y z value of the first point (example: 1 1 1):")
                temp1 = input().split()
                print("Enter the x y z value of the second point (example: 1 1 1):")
                temp2 = input().split()
                print("Enter the rotation angle (in degree):")
                temp3 = input()
                input1 = [temp1, temp2, temp3]

                point1 = input1[0]
                point2 = input1[1]
                angle = int(input1[2])
                xVect = float(point2[0]) - float(point1[0])
                yVect = float(point2[1]) - float(point1[1])
                zVect = float(point2[2]) - float(point1[2])
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
                beta = int(beta)
                miu = int(miu)
                Simulation.transform(self, w, 'trans', x=-1*xVect, y=-1*yVect, z=-1*zVect)
                Simulation.transform(self, w, 'rotY', angle=(beta))
                Simulation.transform(self, w, 'rotX', angle=(-miu))
                Simulation.transform(self, w, 'rotZ', angle=(angle))
                Simulation.transform(self, w, 'rotX', angle=(miu))
                Simulation.transform(self, w, 'rotY', angle=(-beta))
                Simulation.transform(self, w, 'trans', x=xVect, y=yVect, z=zVect)

            elif (command == 9):
                sys.exit()


if __name__ == "__main__":
    Simulation().run()

