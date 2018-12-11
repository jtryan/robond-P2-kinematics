## Project: Kinematics Pick & Place

[//]: # (Image References)

[dhdiagram]: ./misc_images/kuka_DH_diagram.png
[rotx]: ./misc_images/rot-x.png
[roty]: ./misc_images/rot-y.png
[rotz]: ./misc_images/rot-z.png
[dhmatrix]: ./misc_images/DH.png
[toee]: ./misc_images/to-ee.png
[run]:  ./misc_images/inprocess.jpg
[drop]: ./misc_images/drop.jpg
[t0_1]: ./misc_images/t0_1.png
[t1_2]: ./misc_images/t1_2.png
[t2_3]: ./misc_images/t2_3.png
[t3_4]: ./misc_images/t3_4.png
[t4_5]: ./misc_images/t4_5.png
[t5_6]: ./misc_images/t5_6.png
[t6_g]: ./misc_images/t6_g.png
[sideA]: ./misc_images/sideA.png

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/angles.png
[image3]: ./misc_images/misc2.png

## [Forward Kinematics

![Arm running][run]

The project involved performing forward kinematic and reverse kinematic equations to run a `Kuka KR210` simulator that would pick up objects in random locations and drop them into a receptacle. This task is accompllished by performing a series of steps for analysis and implementation. 

The first step was to analyze the `Kuka KR210` robot model description file, `kr210.urdf.xacro` to develop a `Denavit-Hartenberg` table of parameters to use in the Forward kinematics transformations. The xacro file contains information about the links which is used to derive the DH table.

DH diagram from Udacity

![DH Diagram][dhdiagram]

From this information the DH paramter table was derived. The values correspond to the following paramters. 
1. `alpha` is the twist anlge around the joint,
2. `a` is the link length.
3. `d` is the link offset.
4. `q` is theta.

The following table was derived thusly.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 |-0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0



The forward kinematics problem is easily solved by placing the values from the DH parameters into a tranformation matrix for each link. The links are then multiplied together to obtain the homogneous transformation matrix form the base link to the End Effector. The equation for finding the homogenous transforms for each link is 

![Homognenous Transform matrix][dhmatrix]

Placing this table into code results in the following dictionary.

```python
s = {alpha0:      0, a0:      0,  d1:  0.75,  q1:        q1,
    alpha1: -pi/2.,  a1:   0.35,  d2:     0,  q2:  q2-pi/2.,
    alpha2:       0, a2:   1.25,  d3:     0,  q3:        q3,
    alpha3: -pi/2.,  a3: -0.054,  d4:  1.50,  q4:        q4,
    alpha4:  pi/2.,  a4:      0,  d5:     0,  q5:        q5,
    alpha5: -pi/2.,  a5:      0,  d6:     0,  q6:        q6,
    alpha6:      0,  a6:      0,  d7: 0.303,  q7:         0}
```
The transformation matrix was implemented as so.

```python
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[            cos(q),           -sin(q),           0,             a],
            [      sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
            [      sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
            [                      0,                 0,           0,             1]])
    return TF
```

The values for each row in the above table are evaluated in the matrix creating 7 matrices. These values are then used to obtain the final transformation matrix from base to End Effector.

![Base to EE equation][toee]

The matrices for each joint are shown below. 

Joint 1 `T0_1`

![T0_1 Matrix][t0_1]




Joint 2 `T1_2`

![T1_2 Matrix][t1_2]



Joint 3 `T2_3`

![T2_3 Matrix][t2_3]


Joint 4 `T3_4`

![T3_4 Matrix][t3_4]



Joint 5 `T4_5`

![T4_5 Matrix][t4_5]


Joint 6 `T5_6`

![t5_6 Matrix][t5_6]


End Effector `T6_G`

![T6_G Matrix][t6_g]


Creating the individual transformation matrices for each link was done with this code.

```python
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s) 
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(s)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(s)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(s)
T6_G = TF_Matrix(alpha6, a6, d7, q7).subs(s)
```

And then deriving the final transformation matrix from base to End Effector.


```python
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```


## Inverse Kinematics

To perform the Inverse Kinematics an closed-form solution was used. In order to perform a closed form solution one neccessary qualification is that three joint axes intersect at a single point. The Kuka arm has a sperical wrist such that joints 4, 5, 6 interset at the same point.  A series of steps were performed to solve this problem.

1. I created the DH parameter table as shown above.
2. It is required to find the wrist center in relation to the base.
3. Find joint variables q1, q2, q3.
4. Calculate the rotation matrix for the joints
5. Find a set of Euler angles corresponding to the rotation matrix
6. Choose the correct solution

Rotation matrices were used for each of the axes x, y, and z. They represent the roll, pitch and yaw respectively.

R_x

![Rotation Matrix X][rotx]

R_y

![Rotation Matrix Y][roty]

R_z

![Rotation Matrix Z][rotz]


Using these equations for rotation, and the rotation of the End Effector was discovered then the error was applied to this matrix.

```python
 # Compensate for rotation discrepancy between DH parameters and Gazebo
ROT_EE = R_z * R_y * R_x
Rot_Error = R_z.subs(y, radians(180)) * R_y.subs(p, radians(-90))
ROT_EE = ROT_EE * Rot_Error
ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw}

```

Using the values provide for px, py, pz a matrix was created. The Wrist Center was derived from this matrix by subtracting the d value from the last link and multiplying it by the rotation of the End Effector.

```python
 # Calculate joint angles using Geometric IK method
EE = Matrix([[px],
            [py],
            [pz]])

# Calculate the wrist center position
WC = EE - (0.303) * ROT_EE[:,2]
```

![theta1-3][image2]


Theta1 between WC and x is derived by taking the arctangent of WCy and WCx.

```python
theta1 = atan2(WC[1], WC[0])
```

WC is an array of [x, y, z]. Side A is derived from the link lenth and link offset from 3 to 4. Side B is the link length from 2 to 3.

![Side A][sideA]

```python
side_a = 1.501
side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
side_c = 1.25
```

Using the law of cosines the other angles can be calculated. 

cosA	=	(-a^2+b^2+c^2)/(2bc)

cosB	=	(a^2-b^2+c^2)/(2ac)	

cosC	=	(a^2+b^2-c^2)/(2ab)

```python
angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))
```

Theta 2 is derived by subtracting angle from pi/2 radians and the angle from x to Side B. Theta B is derived by subtracting angle B along with a vlaue to acoount for the sag in the link to obtain the complementary angle.

```python
theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75,  sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of 0.054m
```

Using the third column of the first 3 link matrices a rotation matrix from base to link three was created. Then taking the transpose of this matrix and multiplying by the rotation matrix of the End Effector a link matrix from 3 to 6 was created. Theta 5 was calculated using values from R3_6 to find the euler angles. Theta 5 gave 2 possible solutions, so theta 4 and theta 6 were calculated based on whether theta 5 was greater than pi or not. 

```python
# Calculate Euler angles from orientation
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={'q1': theta1, 'q2': theta2, 'q3': theta3})

R3_6 = R0_3.transpose() * ROT_EE

theta5 =  atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])

# There are 2 possible solutions for theta5. 
# Depending on theta5 value choose theat4 and theta6
if theta5 > pi:
    theta4 = atan2(-R3_6[2,2], R3_6[0,2])
    theta6 = atan2(R3_6[1,1], -R3_6[1,0])
else:
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```


## Final

Overall I was pleased with the solution. I found that the calculation for the wrist angles was the hardest part as those being off would cause the arm to hit the side of a cylinder or not properly center on the cylinder and drop it. The code provided a working solution 10/10 times. But I felt like the simulation was slow so I tried a couple ideas to speed up the calculation for poses. I tried pulling the matrices out into a separate class but it really had little effect. I started to work with converting the calculations to numpy away from sympy. Testing has shown great speed improvement but the arm started missing every time.  I will work on that again in the future. 

![Drop cylinder][drop]





