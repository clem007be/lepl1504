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
#	==> Generation Date: Thu Mar 23 15:37:09 2023
#
#	==> Project name: Remorque
#
#	==> Number of joints: 22
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

    S17 = sin(q[17])
    C17 = cos(q[17])
    S20 = sin(q[20])
    C20 = cos(q[20])
 
# Augmented Joint Position Vectors

    Dz101 = q[10]+s.dpt[1,4]
    Dz143 = q[14]+s.dpt[3,8]
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    RLjpqp2_12 = s.dpt[1,13]*C17+s.dpt[3,13]*S17
    RLjpqp2_32 = -s.dpt[1,13]*S17+s.dpt[3,13]*C17
    ORjpqp2_12 = RLjpqp2_32*qd[17]
    ORjpqp2_32 = -RLjpqp2_12*qd[17]
    Apqpjpqp2_12 = ORjpqp2_32*qd[17]
    Apqpjpqp2_32 = -ORjpqp2_12*qd[17]
    RLjpqp4_12 = s.dpt[1,16]*C20+s.dpt[3,16]*S20
    RLjpqp4_32 = -s.dpt[1,16]*S20+s.dpt[3,16]*C20
    ORjpqp4_12 = RLjpqp4_32*qd[20]
    ORjpqp4_32 = -RLjpqp4_12*qd[20]
    Apqpjpqp4_12 = ORjpqp4_32*qd[20]
    Apqpjpqp4_32 = -ORjpqp4_12*qd[20]
    Jdqd[1] = -Apqpjpqp2_12
    Jdqd[2] = -Apqpjpqp2_32
    Jdqd[3] = 0
    Jdqd[4] = -Apqpjpqp4_12
    Jdqd[5] = -Apqpjpqp4_32
    Jdqd[6] = 0

# Number of continuation lines = 0


