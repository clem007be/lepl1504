#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Sun Mar 12 23:19:40 2023
#
#	==> Project name: Livrable1
#
#	==> Number of joints: 5
#
#	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def cons_jdqd(Jdqd, s):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S5 = sin(q[5])
    C5 = cos(q[5])
    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    RLjdqd1_12 = s.dpt[1,10]*C5+s.dpt[3,10]*S5
    RLjdqd1_32 = -s.dpt[1,10]*S5+s.dpt[3,10]*C5
    ORjdqd1_12 = RLjdqd1_32*qd[5]
    ORjdqd1_32 = -RLjdqd1_12*qd[5]
    Apqpjdqd1_12 = ORjdqd1_32*qd[5]
    Apqpjdqd1_32 = -ORjdqd1_12*qd[5]
    ROjdqd2_12 = C2*C3-S2*S3
    ROjdqd2_32 = -C2*S3-S2*C3
    RLjdqd2_12 = s.dpt[1,5]*C2+s.dpt[3,5]*S2
    RLjdqd2_32 = -s.dpt[1,5]*S2+s.dpt[3,5]*C2
    OMjdqd2_22 = qd[2]+qd[3]
    ORjdqd2_12 = RLjdqd2_32*qd[2]
    ORjdqd2_32 = -RLjdqd2_12*qd[2]
    Apqpjdqd2_12 = ORjdqd2_32*qd[2]
    Apqpjdqd2_32 = -ORjdqd2_12*qd[2]
    RLjdqd2_13 = ROjdqd2_12*s.dpt[1,7]
    RLjdqd2_33 = ROjdqd2_32*s.dpt[1,7]
    ORjdqd2_13 = OMjdqd2_22*RLjdqd2_33
    ORjdqd2_33 = -OMjdqd2_22*RLjdqd2_13
    Apqpjdqd2_13 = Apqpjdqd2_12+OMjdqd2_22*ORjdqd2_33
    Apqpjdqd2_33 = Apqpjdqd2_32-OMjdqd2_22*ORjdqd2_13
    jdqd1 = Apqpjdqd1_12-Apqpjdqd2_13
    jdqd3 = Apqpjdqd1_32-Apqpjdqd2_33
    Jdqd[1] = jdqd1
    Jdqd[2] = jdqd3

# Number of continuation lines = 0


