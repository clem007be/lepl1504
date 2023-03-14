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
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S5 = sin(q[5])
    C5 = cos(q[5])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk1_12 = s.dpt[1,9]*C5+s.dpt[3,9]*S5
    RLlnk1_32 = -s.dpt[1,9]*S5+s.dpt[3,9]*C5
    POlnk1_12 = RLlnk1_12+s.dpt[1,3]
    POlnk1_32 = RLlnk1_32+s.dpt[3,3]
    ORlnk1_12 = qd[5]*RLlnk1_32
    ORlnk1_32 = -qd[5]*RLlnk1_12
    Plnk31 = -POlnk1_32+s.dpt[3,2]
    PPlnk1 = POlnk1_12*POlnk1_12+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = -POlnk1_12/Z1
    e31 = Plnk31/Z1
    Zd1 = -ORlnk1_12*e11-ORlnk1_32*e31

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*(e11*C5-e31*S5)
    fPlnk31 = Flink1*(e11*S5+e31*C5)
    trqlnk5_1_2 = fPlnk11*(s.dpt[3,9]-s.l[3,5])-fPlnk31*(s.dpt[1,9]-s.l[1,5])
    fSlnk11 = Flink1*e11
    fSlnk31 = Flink1*e31
    trqlnk1_1_2 = -fSlnk11*(s.dpt[3,2]-s.l[3,1])-fSlnk31*s.l[1,1]
 
# Symbolic model output

    frc[1,1] = s.frc[1,1]-fSlnk11
    frc[3,1] = s.frc[3,1]-fSlnk31
    trq[2,1] = s.trq[2,1]+trqlnk1_1_2
    frc[1,5] = s.frc[1,5]+fPlnk11
    frc[3,5] = s.frc[3,5]+fPlnk31
    trq[2,5] = s.trq[2,5]+trqlnk5_1_2
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1

# Number of continuation lines = 0


