# uncompyle6 version 3.2.4
# Python bytecode 2.7 (62211)
# Decompiled from: Python 2.7.12 (default, Nov 12 2018, 14:36:49) 
# [GCC 5.4.0 20160609]
# Embedded file name: /mnt/hgfs/RoboND-Kinematics-Project/kuka_arm/scripts/My_TMatrix.py
# Compiled at: 2017-12-26 11:54:58
from sympy import symbols, cos, sin, pi, simplify, sqrt, solve, atan2, acos
from sympy.matrices import Matrix
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
alphax, ax, dx, qx, alphae, ae, le = symbols('alphax ax dx qx alphae ae le')
Px, Py, Pz = symbols('Px Py Pz')
r, roll, p, pitch, y, yaw, Rx, Ry, Rz = symbols('r roll p pitch y yaw Rx Ry Rz')
rw, rx, ry, rz, xef, yef, zef = symbols('rw rx ry rz xef yef zef')
l = symbols('l')

def Tn_m(th, alpha, a, d):
    t = Matrix([[cos(th), -sin(th), 0, a],
     [
      sin(th) * cos(alpha), cos(th) * cos(alpha), -sin(alpha), -sin(alpha) * d],
     [
      sin(th) * sin(alpha), cos(th) * sin(alpha), cos(alpha), cos(alpha) * d],
     [
      0, 0, 0, 1]])
    return t


class My_TMatrix:

    def __init__(self, s):
        global Px
        global Py
        global Pz
        global a0
        global a1
        global a2
        global a3
        global a4
        global a5
        global a6
        global alpha0
        global alpha1
        global alpha2
        global alpha3
        global alpha4
        global alpha5
        global alpha6
        global d1
        global d2
        global d3
        global d4
        global d5
        global d6
        global d7
        global p
        global q1
        global q2
        global q3
        global q4
        global q5
        global q6
        global q7
        global r
        global y
        self.s = s
        self.T0_1 = Tn_m(q1, alpha0, a0, d1).subs(self.s)
        self.T1_2 = Tn_m(q2, alpha1, a1, d2).subs(self.s)
        self.T2_3 = Tn_m(q3, alpha2, a2, d3).subs(self.s)
        self.T3_4 = Tn_m(q4, alpha3, a3, d4).subs(self.s)
        self.T4_5 = Tn_m(q5, alpha4, a4, d5).subs(self.s)
        self.T5_6 = Tn_m(q6, alpha5, a5, d6).subs(self.s)
        self.T6_G = Tn_m(q7, alpha6, a6, d7).subs(self.s)
        self.T0_2 = simplify(self.T0_1 * self.T1_2)
        self.T0_3 = simplify(self.T0_2 * self.T2_3)
        self.T0_4 = simplify(self.T0_3 * self.T3_4)
        self.T0_5 = simplify(self.T0_4 * self.T4_5)
        self.T0_6 = simplify(self.T0_5 * self.T5_6)
        self.T0_G = simplify(self.T0_6 * self.T6_G)
        self.Rx = Matrix([[1, 0, 0],
         [
          0, cos(r), -sin(r)],
         [
          0, sin(r), cos(r)]])
        self.Ry = Matrix([[cos(p), 0, sin(p)],
         [
          0, 1, 0],
         [
          -sin(p), 0, cos(p)]])
        self.Rz = Matrix([[cos(y), -sin(y), 0],
         [
          sin(y), cos(y), 0],
         [
          0, 0, 1]])
        self.R_G = simplify(self.Rz * self.Ry * self.Rx)
        self.R_corr = self.Rz.subs(y, pi) * self.Ry.subs(p, -pi / 2.0)
        self.R_G = self.R_G * self.R_corr
        self.EE = Matrix([[Px],
         [
          Py],
         [
          Pz]])
        self.Wc = self.EE - 0.303 * self.R_G[:, 2]


# global Rx ## Warning: Unused global
# global Ry ## Warning: Unused global
# global Rz ## Warning: Unused global
# global pitch ## Warning: Unused global
# global roll ## Warning: Unused global
# global yaw ## Warning: Unused global
# okay decompiling My_TMatrix.pyc
