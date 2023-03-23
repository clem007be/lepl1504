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
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S18 = sin(q[18])
    C18 = cos(q[18])
    S21 = sin(q[21])
    C21 = cos(q[21])
 
# Augmented Joint Position Vectors

    Dz101 = q[10]+s.dpt[1,4]
    Dz143 = q[14]+s.dpt[3,8]
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk1_12 = s.dpt[1,14]*C18
    RLlnk1_32 = -s.dpt[1,14]*S18
    POlnk1_12 = RLlnk1_12+s.dpt[1,12]
    POlnk1_32 = RLlnk1_32+s.dpt[3,12]
    ORlnk1_12 = qd[18]*RLlnk1_32
    ORlnk1_32 = -qd[18]*RLlnk1_12
    Plnk11 = -POlnk1_12+s.dpt[1,13]
    Plnk31 = -POlnk1_32+s.dpt[3,13]
    PPlnk1 = Plnk11*Plnk11+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e31 = Plnk31/Z1
    Zd1 = -ORlnk1_12*e11-ORlnk1_32*e31
    RLlnk3_12 = s.dpt[1,17]*C21
    RLlnk3_32 = -s.dpt[1,17]*S21
    POlnk3_12 = RLlnk3_12+s.dpt[1,15]
    POlnk3_32 = RLlnk3_32+s.dpt[3,15]
    ORlnk3_12 = qd[21]*RLlnk3_32
    ORlnk3_32 = -qd[21]*RLlnk3_12
    Plnk12 = -POlnk3_12+s.dpt[1,16]
    Plnk32 = -POlnk3_32+s.dpt[3,16]
    PPlnk2 = Plnk12*Plnk12+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e12 = Plnk12/Z2
    e32 = Plnk32/Z2
    Zd2 = -ORlnk3_12*e12-ORlnk3_32*e32

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
    Flink2 = s.user_LinkForces(Z2,Zd2,s,tsim,2)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*(e11*C18-e31*S18)
    fPlnk31 = Flink1*(e11*S18+e31*C18)
    trqlnk18_1_2 = -fPlnk31*s.dpt[1,14]
    fSlnk11 = Flink1*e11
    fSlnk31 = Flink1*e31
    trqlnk17_1_2 = -fSlnk11*s.dpt[3,13]+fSlnk31*s.dpt[1,13]
    fPlnk12 = Flink2*(e12*C21-e32*S21)
    fPlnk32 = Flink2*(e12*S21+e32*C21)
    trqlnk21_2_2 = -fPlnk32*s.dpt[1,17]
    fSlnk12 = Flink2*e12
    fSlnk32 = Flink2*e32
    trqlnk20_2_2 = -fSlnk12*s.dpt[3,16]+fSlnk32*s.dpt[1,16]
 
# Symbolic model output

    frc[1,17] = s.frc[1,17]-fSlnk11
    frc[3,17] = s.frc[3,17]-fSlnk31
    trq[2,17] = s.trq[2,17]+trqlnk17_1_2
    frc[1,18] = s.frc[1,18]+fPlnk11
    frc[3,18] = s.frc[3,18]+fPlnk31
    trq[2,18] = s.trq[2,18]+trqlnk18_1_2
    frc[1,20] = s.frc[1,20]-fSlnk12
    frc[3,20] = s.frc[3,20]-fSlnk32
    trq[2,20] = s.trq[2,20]+trqlnk20_2_2
    frc[1,21] = s.frc[1,21]+fPlnk12
    frc[3,21] = s.frc[3,21]+fPlnk32
    trq[2,21] = s.trq[2,21]+trqlnk21_2_2
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1
    Z[2] = Z2
    Zd[2] = Zd2
    Flink[2] = Flink2

# Number of continuation lines = 0


