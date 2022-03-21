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

    def rotateArb(self, x, y, z, xp, yp, zp, angle):
        """Rotasi terhadap titik tertentu"""
        value = np.array([self.x, self.y, self.z, 1])
        if x != 0 or y != 0 or z != 0 or xp != 0 or yp != 0 or zp != 0 :
            X,Y,Z = xp-x, yp-y, zp-z
            a = X/math.sqrt(X**2+Y**2+Z**2)
            b = Y/math.sqrt(X**2+Y**2+Z**2)
            c = Z/math.sqrt(X**2+Y**2+Z**2)
            d = math.sqrt(b**2+c**2)
            rad = angle * math.pi / 180
            cosa = math.cos(rad)
            sina = math.sin(rad)
            #matriks Tranlasi 
            Tx = np.array([ [1, 0, 0, -X],
                            [0, 1, 0, -Y],
                            [0, 0, 1, -Z],
                            [0, 0, 0, 1]])
            Tiv = np.array([[1, 0, 0, X],
                            [0, 1, 0, Y],
                            [0, 0, 1, Z],
                            [0, 0, 0, 1]])
            Rx = np.array([ [1, 0, 0, 0],
                            [0, c/d, -b/d, 0],
                            [0, b/d, c/d, 0],
                            [0, 0, 0, 1]])
            RxIv = np.array([[1, 0, 0, 0],
                            [0, c/d, b/d, 0],
                            [0,-b/d, c/d, 0],
                            [0, 0, 0, 1]])
            Ry = np.array([ [d, 0, -a, 0],
                            [0, 1, 0, 0],
                            [a, 0, d, 0],
                            [0, 0, 0, 1]])
            RyIv = np.array([[d, 0, -a, 0],
                            [0, 1, 0, 0],
                            [-a, 0, d, 0],
                            [0, 0, 0, 1]])
            Rz = np.array([ [cosa,-sina, 0, 0],
                            [sina,cosa, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
            va = np.dot(Tiv, RxIv)
            vb = np.dot(va, RyIv)
            vc = np.dot(vb, Rz)
            vd = np.dot(vc, Ry)
            ve = np.dot(vd, Rx)
            mtx = np.dot(ve, Tx)
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

    def scaling(self, x, y, z):
        """ Translasi sejauh x y dan z """
        mtx = np.array([[x, 0, 0, 0],
                        [0, y, 0, 0],
                        [0, 0, z, 0],
                        [0, 0, 0, 1]])
        value = np.dot(mtx, [self.x, self.y, self.z, 1])
        return Point3D(value[0], value[1], value[2])

    def shearing(self, x, y, z):
        """ Shearing sejauh xy yz atau zx """
        # get the shearing matrices
        mtx = [[0]*4]*4
        if(x == 0):
            mtx = np.array([[1, 0, 0, 0],
                            [y, 1, 0, 0],
                            [z, 0, 1, 0],
                            [0, 0, 0, 1]])
        elif(y == 0):
            mtx = np.array([[1, x, 0, 0],
                            [0, 1, 0, 0],
                            [0, z, 1, 0],
                            [0, 0, 0, 1]])
        elif(z == 0):
            mtx = np.array([[1, 0, x, 0],
                            [0, 1, y, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        else:
            mtx = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
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
            Point3D(-1,1,5),
            Point3D(1,1,5),
            Point3D(1,-1,5),
            Point3D(-1,-1,5)
        ]

        self.faces = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]
        self.angleX, self.angleY, self.angleZ = 0, 0, 0
        self.trX, self.trY, self.trZ = 0, 0, 0
        self.scX, self.scY, self.scZ = 1, 1, 1
        self.shX, self.shY, self.shZ = 1, 1, 1
        self.arbX, self.arbY, self.arbZ = 0, 0, 0
        self.arbXP, self.arbYP, self.arbZP = 0, 0, 0
        self.cond = 0

        self.l1 = [[],[],[],[],[],[]]
        self.l2 = [[],[],[],[],[],[]]
        self.l3 = [[],[],[],[],[],[]]
        self.l4 = [[],[],[],[],[],[]]

    def transformation(self, w, angle, mode, changeX = 0, changeY = 0, changeZ = 0, xp = 0, yp = 0, zp = 0):
        # Inisialisasi Nilai
        x, y, z = 0, 0, 0
        if mode == 'rotX' :
            x = 1
            if angle < 0:
                x = -1 
        elif mode == 'rotY' :
            y = 1
            if angle < 0:
                y = -1 
        elif mode == 'rotZ':
            z = 1
            if angle < 0:
                z = -1 
        elif mode == 'scall':
            self.scX *= changeX
            self.scY *= changeY
            self.scZ *= changeZ
        elif mode == 'shear':
            self.shX = changeX
            self.shY = changeY
            self.shZ = changeZ
        elif mode == 'rotArb':
            self.arbX = changeX
            self.arbY = changeY
            self.arbZ = changeZ
            self.arbXP = xp
            self.arbYP = yp
            self.arbZP = zp

        # Iterasi Transformasi
        for i in range(abs(angle)):
            sleep(0.01)

            # Animasi
            if mode == 'rotX' or mode == 'rotY' or mode == 'rotZ':
                self.angleX += x
                self.angleY += y
                self.angleZ += z
            elif mode == 'trans': 
                self.trX += changeX/angle
                self.trY += changeY/angle
                self.trZ += changeZ/angle

            # Will hold transformed vertices.
            t = []
            gx = 0
            
            for v in self.vertices:
                # Transformasi
                r = v.rotateX(self.angleX).rotateY(self.angleY).rotateZ(self.angleZ).translation(self.trX, self.trY, self.trZ).scaling(self.scX, self.scY, self.scZ).shearing(self.shX, self.shY, self.shZ).rotateArb(self.arbX, self.arbY, self.arbZ, self.arbXP, self.arbYP, self.arbZP, angle)
                # Digambarkan dalam 2D
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

    def run(self):
        w = GraphWin("Tugas TVG", 640, 480)
        w.setBackground('white')

        Simulation.transformation(self, w, 1, '')
        print("\nBerikut merupakan program sederhana simulasi transformasi 3D sebuah balok\nSecara default balok memiliki sisi rusuk dengan panjang 2 dan pusat 0,0,0")

        lanjut = True
        while lanjut == True:
            print("\nPilih metode transformasi yang ingin dilakukan : \n1. Translasi \n2. Scaling \n3. Rotasi \n4. Shearing \n5. Rotasi Arbitrari Axis")
            methode = int(input("Pilih sesuai nomor \n"))
            if methode == 1:
                print("Anda memilih metode translasi.")
                x = float(input("Geser x sejauh : "))
                y = float(input("Geser y sejauh : "))
                z = float(input("Geser z sejauh : "))
                Simulation.transformation(self, w, 10, 'trans', x, y, z)

            elif methode == 2:
                print("Anda memilih metode scaling.")
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

            elif methode == 4:
                print("Anda memilih metode shearing.\n")
                print("Tentukan sumbu shear yang ingin dilakukan : \n1. xy \n2. yz \n3. xz")
                shear = int(input("Pilih sesuai nomor \n"))
                if shear == 1 or shear == 3:
                    x = int(input("Shear x sebesar : "))
                if shear == 1 or shear == 2:
                    y = int(input("Shear y sebesar : "))
                if shear == 2 or shear == 3:
                    z = int(input("Shear z sebesar : "))

                if shear == 1:
                    Simulation.transformation(self, w, 1, 'shear', x, y, 0)
                elif shear == 2:
                    Simulation.transformation(self, w, 1, 'shear', 0, y, z)
                elif shear == 3:
                    Simulation.transformation(self, w, 1, 'shear', x, 0, z)
            
            elif methode == 5:
                print("Anda memilih metode rotasi terhadap arbitrari axis.\n")
                print("Masukan dua koordinat untuk garis axis rotasi: ")
                print("Masukkan koordinat titik ke-1")
                xo=(int(input("X = ")))
                yo=(int(input("Y = ")))
                zo=(int(input("Z = ")))
                print("Masukkan koordinat titik ke-2")
                xp=(int(input("X = ")))
                yp=(int(input("Y = ")))
                zp=(int(input("Z = ")))
                degree = int(input("Tentukan sudut putar (dalam satuan derajat): "))
                Simulation.transformation(self, w, 1, 'rotArb', xo, yo, zo, xp, yp, zp)

            state = input("Input Transformasi lain? (y/n): ")
            if state == "y":
                lanjut = True
            elif state == "n":
                lanjut = False

        input('Press ENTER to exit')

Simulation().run()