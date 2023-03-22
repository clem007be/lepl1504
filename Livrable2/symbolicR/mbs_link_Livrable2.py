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
#	==> Generation Date: Thu Mar 23 00:40:48 2023
#
#	==> Project name: Livrable2
#
#	==> Number of joints: 10
#
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    ROlnk2_19 = C8*C9
    ROlnk2_39 = -S8*C9
    ROlnk2_49 = -C8*S9
    ROlnk2_69 = S8*S9
    OMlnk2_12 = qd[9]*S8
    OMlnk2_32 = qd[9]*C8
    RLlnk2_13 = ROlnk2_49*s.dpt[2,8]
    RLlnk2_23 = s.dpt[2,8]*C9
    RLlnk2_33 = ROlnk2_69*s.dpt[2,8]
    POlnk2_13 = RLlnk2_13+s.dpt[1,3]
    POlnk2_33 = RLlnk2_33+s.dpt[3,3]
    ORlnk2_13 = qd[8]*RLlnk2_33-OMlnk2_32*RLlnk2_23
    ORlnk2_23 = -OMlnk2_12*RLlnk2_33+OMlnk2_32*RLlnk2_13
    ORlnk2_33 = -qd[8]*RLlnk2_13+OMlnk2_12*RLlnk2_23
    Plnk31 = POlnk2_33-s.dpt[3,4]
    PPlnk1 = POlnk2_13*POlnk2_13+Plnk31*Plnk31+RLlnk2_23*RLlnk2_23
    Z1 = sqrt(PPlnk1)
    e11 = POlnk2_13/Z1
    e21 = RLlnk2_23/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_13*e11+ORlnk2_23*e21+ORlnk2_33*e31

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*e11
    fPlnk21 = Flink1*e21
    fPlnk31 = Flink1*e31
    trqlnk6_1_1 = -fPlnk21*(s.dpt[3,4]-s.l[3,6])
    trqlnk6_1_2 = fPlnk11*(s.dpt[3,4]-s.l[3,6])+fPlnk31*s.l[1,6]
    trqlnk6_1_3 = -fPlnk21*s.l[1,6]
    fSlnk11 = Flink1*(ROlnk2_19*e11+ROlnk2_39*e31+e21*S9)
    fSlnk21 = Flink1*(ROlnk2_49*e11+ROlnk2_69*e31+e21*C9)
    fSlnk31 = Flink1*(e11*S8+e31*C8)
    trqlnk9_1_1 = -fSlnk21*s.l[3,9]-fSlnk31*s.dpt[2,8]
    trqlnk9_1_2 = fSlnk11*s.l[3,9]
    trqlnk9_1_3 = fSlnk11*s.dpt[2,8]
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+fPlnk11
    frc[2,6] = s.frc[2,6]+fPlnk21
    frc[3,6] = s.frc[3,6]+fPlnk31
    trq[1,6] = s.trq[1,6]+trqlnk6_1_1
    trq[2,6] = s.trq[2,6]+trqlnk6_1_2
    trq[3,6] = s.trq[3,6]+trqlnk6_1_3
    frc[1,9] = s.frc[1,9]-fSlnk11
    frc[2,9] = s.frc[2,9]-fSlnk21
    frc[3,9] = s.frc[3,9]-fSlnk31
    trq[1,9] = s.trq[1,9]+trqlnk9_1_1
    trq[2,9] = s.trq[2,9]+trqlnk9_1_2
    trq[3,9] = s.trq[3,9]+trqlnk9_1_3
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1

# Number of continuation lines = 0


