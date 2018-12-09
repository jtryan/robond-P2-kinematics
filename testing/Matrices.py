

from sympy import symbols, cos, sin, pi, simplify, sqrt, solve, atan2, acos
from sympy.matrices import Matrix
from mpmath import radians


q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offset
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link length
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angle
R_z, R_Y, R_X = symbols('R_z R_y R_x')
r, p, y = symbols('r p y')
Px, Py, Pz = symbols('Px Py Pz')

def TF_Matrix(alpha, a, d, q):
    TM = Matrix([[            cos(q),           -sin(q),           0,             a],
            [      sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
            [      sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),   cos(alpha)*d],
            [                      0,                 0,           0,             1]])
    return TM


class Matrices:
    def __init__(self, s):
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
        global q1
        global q2
        global q3
        global q4
        global q5
        global q6
        global q7
        global r
        global p
        global y
    
        global Px
        global Py
        global Pz
        

        self.s = s

        # Create individual transformation matrices
        self.T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(self.s)
        self.T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(self.s)
        self.T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(self.s) 
        self.T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(self.s)
        self.T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(self.s)
        self.T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(self.s)
        self.T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(self.s)

            
        self.R_x = Matrix([[ 1,              0,        0],
                            [ 0,        sin(r),  cos(r)],   #ROLL
                            [ 0,        cos(r), -sin(r)]])
                    

        self.R_y = Matrix([[ cos(p),        0,  sin(p)],
                        [       0,        1,        0],
                        [-sin(p),        0,  cos(p)]])    #PITCH

        self.R_z = Matrix([[ cos(y), -sin(y),        0],
                        [ sin(y),  cos(y),        0],
                        [ 0,              0,        1]])   # YAW

        self.T0_EE = self.T0_1 * self.T1_2 * self.T2_3 * self.T3_4 * self.T4_5 * self.T5_6 * self.T6_EE

        self.ROT_EE = self.R_z * self.R_y * self.R_x
        self.Rot_Error = self.R_z.subs(y, radians(180)) * self.R_y.subs(p, radians(-90))
        self.ROT_EE = self.ROT_EE * self.Rot_Error
        self.EE = Matrix([[Px],
                          [Py],
                          [Pz]])

            # Set wrist variables
        self.WC = self.EE - (0.303) * self.ROT_EE[:,2]
        


