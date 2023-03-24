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
#	==> Generation Date: Fri Mar 24 08:11:21 2023
#
#	==> Project name: Remorque1
#
#	==> Number of joints: 17
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

    S13 = sin(q[13])
    C13 = cos(q[13])
    S15 = sin(q[15])
    C15 = cos(q[15])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_12 = s.dpt[1,13]*C13+s.dpt[3,13]*S13
    RLlnk2_32 = -s.dpt[1,13]*S13+s.dpt[3,13]*C13
    POlnk2_12 = RLlnk2_12+s.dpt[1,7]
    POlnk2_32 = RLlnk2_32+s.dpt[3,7]
    ORlnk2_12 = qd[13]*RLlnk2_32
    ORlnk2_32 = -qd[13]*RLlnk2_12
    Plnk11 = POlnk2_12-s.dpt[1,9]
    Plnk21 = s.dpt[2,7]-s.dpt[2,9]
    Plnk31 = POlnk2_32-s.dpt[3,9]
    PPlnk1 = Plnk11*Plnk11+Plnk21*Plnk21+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e21 = Plnk21/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_12*e11+ORlnk2_32*e31
    RLlnk4_12 = s.dpt[1,16]*C15+s.dpt[3,16]*S15
    RLlnk4_32 = -s.dpt[1,16]*S15+s.dpt[3,16]*C15
    POlnk4_12 = RLlnk4_12+s.dpt[1,8]
    POlnk4_32 = RLlnk4_32+s.dpt[3,8]
    ORlnk4_12 = qd[15]*RLlnk4_32
    ORlnk4_32 = -qd[15]*RLlnk4_12
    Plnk12 = POlnk4_12-s.dpt[1,10]
    Plnk22 = -s.dpt[2,10]+s.dpt[2,8]
    Plnk32 = POlnk4_32-s.dpt[3,10]
    PPlnk2 = Plnk12*Plnk12+Plnk22*Plnk22+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e12 = Plnk12/Z2
    e22 = Plnk22/Z2
    e32 = Plnk32/Z2
    Zd2 = ORlnk4_12*e12+ORlnk4_32*e32

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
    Flink2 = s.user_LinkForces(Z2,Zd2,s,tsim,2)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*e11
    fPlnk21 = Flink1*e21
    fPlnk31 = Flink1*e31
    trqlnk12_1_1 = -fPlnk21*s.dpt[3,9]+fPlnk31*s.dpt[2,9]
    trqlnk12_1_2 = fPlnk11*s.dpt[3,9]-fPlnk31*(s.dpt[1,9]-s.l[1,12])
    trqlnk12_1_3 = -fPlnk11*s.dpt[2,9]+fPlnk21*(s.dpt[1,9]-s.l[1,12])
    fSlnk11 = Flink1*(e11*C13-e31*S13)
    fSlnk21 = Flink1*e21
    fSlnk31 = Flink1*(e11*S13+e31*C13)
    trqlnk13_1_1 = fSlnk21*s.dpt[3,13]
    trqlnk13_1_2 = -fSlnk11*s.dpt[3,13]+fSlnk31*(s.dpt[1,13]-s.l[1,13])
    trqlnk13_1_3 = -fSlnk21*(s.dpt[1,13]-s.l[1,13])
    fPlnk12 = Flink2*e12
    fPlnk22 = Flink2*e22
    fPlnk32 = Flink2*e32
    frclnk12_2_1 = fPlnk11+fPlnk12
    frclnk12_2_2 = fPlnk21+fPlnk22
    frclnk12_2_3 = fPlnk31+fPlnk32
    trqlnk12_2_1 = trqlnk12_1_1-fPlnk22*s.dpt[3,10]+fPlnk32*s.dpt[2,10]
    trqlnk12_2_2 = trqlnk12_1_2+fPlnk12*s.dpt[3,10]-fPlnk32*(s.dpt[1,10]-s.l[1,12])
    trqlnk12_2_3 = trqlnk12_1_3-fPlnk12*s.dpt[2,10]+fPlnk22*(s.dpt[1,10]-s.l[1,12])
    fSlnk12 = Flink2*(e12*C15-e32*S15)
    fSlnk22 = Flink2*e22
    fSlnk32 = Flink2*(e12*S15+e32*C15)
    trqlnk15_2_1 = fSlnk22*s.dpt[3,16]
    trqlnk15_2_2 = -fSlnk12*s.dpt[3,16]+fSlnk32*(s.dpt[1,16]-s.l[1,15])
    trqlnk15_2_3 = -fSlnk22*(s.dpt[1,16]-s.l[1,15])
 
# Symbolic model output

    frc[1,12] = s.frc[1,12]+frclnk12_2_1
    frc[2,12] = s.frc[2,12]+frclnk12_2_2
    frc[3,12] = s.frc[3,12]+frclnk12_2_3
    trq[1,12] = s.trq[1,12]+trqlnk12_2_1
    trq[2,12] = s.trq[2,12]+trqlnk12_2_2
    trq[3,12] = s.trq[3,12]+trqlnk12_2_3
    frc[1,13] = s.frc[1,13]-fSlnk11
    frc[2,13] = s.frc[2,13]-fSlnk21
    frc[3,13] = s.frc[3,13]-fSlnk31
    trq[1,13] = s.trq[1,13]+trqlnk13_1_1
    trq[2,13] = s.trq[2,13]+trqlnk13_1_2
    trq[3,13] = s.trq[3,13]+trqlnk13_1_3
    frc[1,15] = s.frc[1,15]-fSlnk12
    frc[2,15] = s.frc[2,15]-fSlnk22
    frc[3,15] = s.frc[3,15]-fSlnk32
    trq[1,15] = s.trq[1,15]+trqlnk15_2_1
    trq[2,15] = s.trq[2,15]+trqlnk15_2_2
    trq[3,15] = s.trq[3,15]+trqlnk15_2_3
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1
    Z[2] = Z2
    Zd[2] = Zd2
    Flink[2] = Flink2

# Number of continuation lines = 0


