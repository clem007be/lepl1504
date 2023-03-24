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
#	==> Generation Date: Fri Mar 24 13:37:37 2023
#
#	==> Project name: Remorque1
#
#	==> Number of joints: 16
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

    S10 = sin(q[10])
    C10 = cos(q[10])
    S12 = sin(q[12])
    C12 = cos(q[12])
 
# Augmented Joint Position Vectors

    Dz153 = q[15]+s.dpt[3,4]
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_12 = s.dpt[1,12]*C10+s.dpt[3,12]*S10
    RLlnk2_32 = -s.dpt[1,12]*S10+s.dpt[3,12]*C10
    POlnk2_12 = RLlnk2_12+s.dpt[1,6]
    POlnk2_32 = RLlnk2_32+s.dpt[3,6]
    ORlnk2_12 = qd[10]*RLlnk2_32
    ORlnk2_32 = -qd[10]*RLlnk2_12
    Plnk11 = POlnk2_12-s.dpt[1,8]
    Plnk21 = s.dpt[2,6]-s.dpt[2,8]
    Plnk31 = POlnk2_32-s.dpt[3,8]
    PPlnk1 = Plnk11*Plnk11+Plnk21*Plnk21+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e21 = Plnk21/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_12*e11+ORlnk2_32*e31
    RLlnk4_12 = s.dpt[1,15]*C12+s.dpt[3,15]*S12
    RLlnk4_32 = -s.dpt[1,15]*S12+s.dpt[3,15]*C12
    POlnk4_12 = RLlnk4_12+s.dpt[1,7]
    POlnk4_32 = RLlnk4_32+s.dpt[3,7]
    ORlnk4_12 = qd[12]*RLlnk4_32
    ORlnk4_32 = -qd[12]*RLlnk4_12
    Plnk12 = POlnk4_12-s.dpt[1,9]
    Plnk22 = s.dpt[2,7]-s.dpt[2,9]
    Plnk32 = POlnk4_32-s.dpt[3,9]
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
    trqlnk9_1_1 = -fPlnk21*s.dpt[3,8]+fPlnk31*s.dpt[2,8]
    trqlnk9_1_2 = fPlnk11*s.dpt[3,8]-fPlnk31*(s.dpt[1,8]-s.l[1,9])
    trqlnk9_1_3 = -fPlnk11*s.dpt[2,8]+fPlnk21*(s.dpt[1,8]-s.l[1,9])
    fSlnk11 = Flink1*(e11*C10-e31*S10)
    fSlnk21 = Flink1*e21
    fSlnk31 = Flink1*(e11*S10+e31*C10)
    trqlnk10_1_1 = fSlnk21*s.dpt[3,12]
    trqlnk10_1_2 = -fSlnk11*s.dpt[3,12]+fSlnk31*(s.dpt[1,12]-s.l[1,10])
    trqlnk10_1_3 = -fSlnk21*(s.dpt[1,12]-s.l[1,10])
    fPlnk12 = Flink2*e12
    fPlnk22 = Flink2*e22
    fPlnk32 = Flink2*e32
    frclnk9_2_1 = fPlnk11+fPlnk12
    frclnk9_2_2 = fPlnk21+fPlnk22
    frclnk9_2_3 = fPlnk31+fPlnk32
    trqlnk9_2_1 = trqlnk9_1_1-fPlnk22*s.dpt[3,9]+fPlnk32*s.dpt[2,9]
    trqlnk9_2_2 = trqlnk9_1_2+fPlnk12*s.dpt[3,9]-fPlnk32*(s.dpt[1,9]-s.l[1,9])
    trqlnk9_2_3 = trqlnk9_1_3-fPlnk12*s.dpt[2,9]+fPlnk22*(s.dpt[1,9]-s.l[1,9])
    fSlnk12 = Flink2*(e12*C12-e32*S12)
    fSlnk22 = Flink2*e22
    fSlnk32 = Flink2*(e12*S12+e32*C12)
    trqlnk12_2_1 = fSlnk22*s.dpt[3,15]
    trqlnk12_2_2 = -fSlnk12*s.dpt[3,15]+fSlnk32*(s.dpt[1,15]-s.l[1,12])
    trqlnk12_2_3 = -fSlnk22*(s.dpt[1,15]-s.l[1,12])
 
# Symbolic model output

    frc[1,9] = s.frc[1,9]+frclnk9_2_1
    frc[2,9] = s.frc[2,9]+frclnk9_2_2
    frc[3,9] = s.frc[3,9]+frclnk9_2_3
    trq[1,9] = s.trq[1,9]+trqlnk9_2_1
    trq[2,9] = s.trq[2,9]+trqlnk9_2_2
    trq[3,9] = s.trq[3,9]+trqlnk9_2_3
    frc[1,10] = s.frc[1,10]-fSlnk11
    frc[2,10] = s.frc[2,10]-fSlnk21
    frc[3,10] = s.frc[3,10]-fSlnk31
    trq[1,10] = s.trq[1,10]+trqlnk10_1_1
    trq[2,10] = s.trq[2,10]+trqlnk10_1_2
    trq[3,10] = s.trq[3,10]+trqlnk10_1_3
    frc[1,12] = s.frc[1,12]-fSlnk12
    frc[2,12] = s.frc[2,12]-fSlnk22
    frc[3,12] = s.frc[3,12]-fSlnk32
    trq[1,12] = s.trq[1,12]+trqlnk12_2_1
    trq[2,12] = s.trq[2,12]+trqlnk12_2_2
    trq[3,12] = s.trq[3,12]+trqlnk12_2_3
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1
    Z[2] = Z2
    Zd[2] = Zd2
    Flink[2] = Flink2

# Number of continuation lines = 0


