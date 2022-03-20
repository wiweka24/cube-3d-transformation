import math
import numpy as np
from graphics import *
from time import sleep

class Point3D:
    def __init__(self, x = 0, y = 0, z = 0):
        self.x, self.y, self.z = float(x), float(y), float(z)
 
    def rotateX(self, angle):
        """ Rotasi dengan sumbu putar sumbu X """
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        mtx = np.array([[1, 0, 0, 0],
                        [0, cosa, -sina, 0],
                        [0, sina, cosa, 0],
                        [0, 0, 0, 1]])
        value = np.dot(mtx, [self.x, self.y, self.z, 1])
        return Point3D(value[0], value[1], value[2])
 
    def rotateY(self, angle):
        """ Rotasi dengan sumbu putar sumbu Y """
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        mtx = np.array([[cosa, 0, sina, 0],
                        [0, 1, 0, 0],
                        [-sina, 0, cosa, 0],
                        [0, 0, 0, 1]])
        value = np.dot(mtx, [self.x, self.y, self.z, 1])
        return Point3D(value[0], value[1], value[2])
 
    def rotateZ(self, angle):
        """ Rotasi dengan sumbu putar sumbu Z """
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        mtx = np.array([[cosa, -sina, 0, 0],
                        [sina, cosa, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        value = np.dot(mtx, [self.x, self.y, self.z, 1])
        return Point3D(value[0], value[1], value[2])

    def translation(self, x, y, z):
        """ Translasi sejauh x y dan z """
        mtx = np.array([[1, 0, 0, x],
                        [0, 1, 0, y],
                        [0, 0, 1, z],
                        [0, 0, 0, 1]])
        value = np.dot(mtx, [self.x, self.y, self.z, 1])
        return Point3D(value[0], value[1], value[2])

    def scalling(self, x, y, z):
        """ Translasi sejauh x y dan z """
        mtx = np.array([[x, 0, 0, 0],
                        [0, y, 0, 0],
                        [0, 0, z, 0],
                        [0, 0, 0, 1]])
        value = np.dot(mtx, [self.x, self.y, self.z, 1])
        return Point3D(value[0], value[1], value[2])
 
    def project(self, win_width, win_height, fov, viewer_distance):
        """ Transforms this 3D point to 2D using a perspective projection. """
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, 1)

class Simulation:
    def __init__(self):
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

        self.faces = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]
        self.angleX, self.angleY, self.angleZ = 0, 0, 0
        self.chX, self.chY, self.chZ, self.scX, self.scY, self.scZ = 0, 0, 0, 1, 1, 1
        self.cond = 0

        self.l1 = [[],[],[],[],[],[]]
        self.l2 = [[],[],[],[],[],[]]
        self.l3 = [[],[],[],[],[],[]]
        self.l4 = [[],[],[],[],[],[]]

    def transformation(self, w, angle, mode, changeX = 0, changeY = 0, changeZ = 0):
        # Inisialisasi Nilai
        x, y, z = 0, 0, 0
        if mode == 'rotX' :
            x = 1
        elif mode == 'rotY' :
            y = 1
        elif mode == 'rotZ':
            z = 1
        elif mode == 'scall':
            self.scX += changeX
            self.scY += changeY
            self.scZ += changeZ
        elif mode == 'trans':
            self.chX += changeX/angle
            self.chY += changeY/angle
            self.chZ += changeZ/angle
        else :
            x, y, z = 0, 0, 0

        # Iterasi Transformasi
        for i in range(angle):
            sleep(0.01)

            # Will hold transformed vertices.
            t = []
            gx = 0
            
            for v in self.vertices:
                # Transformasi
                if mode == 'trans':
                    r = v.translation(self.chX, self.chY, self.chZ)
                elif mode == 'scall':
                    r = v.scalling(self.scX, self.scY, self.scZ)
                else :
                    r = v.rotateX(self.angleX).rotateY(self.angleY).rotateZ(self.angleZ)
                
                p = r.project(640, 480, 200, 4)
                # Put the point in the list of transformed vertices
                t.append(p)

            # Identifikasi apakah sudah pernah transformasi
            if(self.cond == 1):
                for i in range(6):
                    self.l1[i].undraw()
                    self.l2[i].undraw()
                    self.l3[i].undraw()
                    self.l4[i].undraw()
            else:
                self.cond = 1

            # Fungsi untuk menggambar
            for f in self.faces:
                self.l1[gx]=Line(Point(t[f[0]].x, t[f[0]].y), Point(t[f[1]].x, t[f[1]].y))
                self.l2[gx]=Line(Point(t[f[1]].x, t[f[1]].y), Point(t[f[2]].x, t[f[2]].y))
                self.l3[gx]=Line(Point(t[f[2]].x, t[f[2]].y), Point(t[f[3]].x, t[f[3]].y))
                self.l4[gx]=Line(Point(t[f[3]].x, t[f[3]].y), Point(t[f[0]].x, t[f[0]].y))
                self.l1[gx].setFill('black')
                self.l2[gx].setFill('black')
                self.l3[gx].setFill('black')
                self.l4[gx].setFill('black')
                self.l1[gx].draw(w)
                self.l2[gx].draw(w)
                self.l3[gx].draw(w)
                self.l4[gx].draw(w)
                gx += 1
            
            # Animasi
            self.angleX += x
            self.angleY += y
            self.angleZ += z
            self.chX += changeX/angle
            self.chY += changeY/angle
            self.chZ += changeZ/angle
            

    def run(self):
        w = GraphWin("Tugas TVG", 640, 480)
        w.setBackground('white')

        Simulation.transformation(self, w, 1, '', 0, 0, 0)
        print("\nBerikut merupakan program sederhana simulasi transformasi 3D sebuah balok\nSecara default balok memiliki sisi rusuk dengan panjang 2 dan pusta 0,0,0")

        lanjut = True
        while lanjut == True:
            print("\nPilih metode transformasi yang ingin dilakukan : \n1. Translasi \n2. Scaling \n3. Rotasi")
            methode = int(input("Pilih sesuai nomor \n"))
            if methode == 1:
                print("Anda memilih metode translasi.")
                x = float(input("Geser x sejauh : "))
                y = float(input("Geser y sejauh : "))
                z = float(input("Geser z sejauh : "))
                Simulation.transformation(self, w, 10, 'trans', x, y, z)
            elif methode == 2:
                print("Anda memilih metode scalling.")
                x = float(input("scale x sebanyak : "))
                y = float(input("scale y sebanyak : "))
                z = float(input("scale z sebanyak : "))
                Simulation.transformation(self, w, 1, 'scall', x, y, z)
            elif methode == 3:
                print("Anda memilih metode rotasi.")
                print("Tentukan sumbu rotasi yang ingin dilakukan : \n1. x \n2. y \n3. z")
                axis = input("Pilih sesuai nomor atau sumbu: ")
                angle = int(input("Tentukan sudut putar (dalam satuan derajat): "))
                if axis == '1' or axis == 'x':
                    Simulation.transformation(self, w, angle, 'rotX')
                elif axis == '2' or axis == 'y':
                    Simulation.transformation(self, w, angle, 'rotY')
                elif axis == '3' or axis == 'z':
                    Simulation.transformation(self, w, angle, 'rotZ')
            
            state = input("Input Transformasi lain? (y/n): ")
            if state == "y":
                lanjut = True
            elif state == "n":
                lanjut = False

        input('Press ENTER to exit')

Simulation().run()