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
#	==> Generation Date: Fri Mar 24 17:30:30 2023
#
#	==> Project name: Livrable2
#
#	==> Number of joints: 25
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
    S19 = sin(q[19])
    C19 = cos(q[19])
    S21 = sin(q[21])
    C21 = cos(q[21])
 
# Augmented Joint Position Vectors

    Dz243 = q[24]+s.dpt[3,16]
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    ROlnk2_19 = C8*C9
    ROlnk2_39 = -S8*C9
    ROlnk2_49 = -C8*S9
    ROlnk2_69 = S8*S9
    OMlnk2_12 = qd[9]*S8
    OMlnk2_32 = qd[9]*C8
    RLlnk2_13 = ROlnk2_49*s.dpt[2,9]
    RLlnk2_23 = s.dpt[2,9]*C9
    RLlnk2_33 = ROlnk2_69*s.dpt[2,9]
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
    RLlnk4_12 = s.dpt[1,24]*C19+s.dpt[3,24]*S19
    RLlnk4_32 = -s.dpt[1,24]*S19+s.dpt[3,24]*C19
    POlnk4_12 = RLlnk4_12+s.dpt[1,18]
    POlnk4_32 = RLlnk4_32+s.dpt[3,18]
    ORlnk4_12 = qd[19]*RLlnk4_32
    ORlnk4_32 = -qd[19]*RLlnk4_12
    Plnk12 = POlnk4_12-s.dpt[1,20]
    Plnk22 = s.dpt[2,18]-s.dpt[2,20]
    Plnk32 = POlnk4_32-s.dpt[3,20]
    PPlnk2 = Plnk12*Plnk12+Plnk22*Plnk22+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e12 = Plnk12/Z2
    e22 = Plnk22/Z2
    e32 = Plnk32/Z2
    Zd2 = ORlnk4_12*e12+ORlnk4_32*e32
    RLlnk6_12 = s.dpt[1,27]*C21+s.dpt[3,27]*S21
    RLlnk6_32 = -s.dpt[1,27]*S21+s.dpt[3,27]*C21
    POlnk6_12 = RLlnk6_12+s.dpt[1,19]
    POlnk6_32 = RLlnk6_32+s.dpt[3,19]
    ORlnk6_12 = qd[21]*RLlnk6_32
    ORlnk6_32 = -qd[21]*RLlnk6_12
    Plnk13 = POlnk6_12-s.dpt[1,21]
    Plnk23 = s.dpt[2,19]-s.dpt[2,21]
    Plnk33 = POlnk6_32-s.dpt[3,21]
    PPlnk3 = Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33
    Z3 = sqrt(PPlnk3)
    e13 = Plnk13/Z3
    e23 = Plnk23/Z3
    e33 = Plnk33/Z3
    Zd3 = ORlnk6_12*e13+ORlnk6_32*e33

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
    Flink2 = s.user_LinkForces(Z2,Zd2,s,tsim,2)
    Flink3 = s.user_LinkForces(Z3,Zd3,s,tsim,3)
 
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
    trqlnk9_1_1 = -fSlnk21*s.l[3,9]-fSlnk31*s.dpt[2,9]
    trqlnk9_1_2 = fSlnk11*s.l[3,9]
    trqlnk9_1_3 = fSlnk11*s.dpt[2,9]
    fPlnk12 = Flink2*e12
    fPlnk22 = Flink2*e22
    fPlnk32 = Flink2*e32
    trqlnk18_2_1 = -fPlnk22*s.dpt[3,20]+fPlnk32*s.dpt[2,20]
    trqlnk18_2_2 = fPlnk12*s.dpt[3,20]-fPlnk32*(s.dpt[1,20]-s.l[1,18])
    trqlnk18_2_3 = -fPlnk12*s.dpt[2,20]+fPlnk22*(s.dpt[1,20]-s.l[1,18])
    fSlnk12 = Flink2*(e12*C19-e32*S19)
    fSlnk22 = Flink2*e22
    fSlnk32 = Flink2*(e12*S19+e32*C19)
    trqlnk19_2_1 = fSlnk22*s.dpt[3,24]
    trqlnk19_2_2 = -fSlnk12*s.dpt[3,24]+fSlnk32*(s.dpt[1,24]-s.l[1,19])
    trqlnk19_2_3 = -fSlnk22*(s.dpt[1,24]-s.l[1,19])
    fPlnk13 = Flink3*e13
    fPlnk23 = Flink3*e23
    fPlnk33 = Flink3*e33
    frclnk18_3_1 = fPlnk12+fPlnk13
    frclnk18_3_2 = fPlnk22+fPlnk23
    frclnk18_3_3 = fPlnk32+fPlnk33
    trqlnk18_3_1 = trqlnk18_2_1-fPlnk23*s.dpt[3,21]+fPlnk33*s.dpt[2,21]
    trqlnk18_3_2 = trqlnk18_2_2+fPlnk13*s.dpt[3,21]-fPlnk33*(s.dpt[1,21]-s.l[1,18])
    trqlnk18_3_3 = trqlnk18_2_3-fPlnk13*s.dpt[2,21]+fPlnk23*(s.dpt[1,21]-s.l[1,18])
    fSlnk13 = Flink3*(e13*C21-e33*S21)
    fSlnk23 = Flink3*e23
    fSlnk33 = Flink3*(e13*S21+e33*C21)
    trqlnk21_3_1 = fSlnk23*s.dpt[3,27]
    trqlnk21_3_2 = -fSlnk13*s.dpt[3,27]+fSlnk33*(s.dpt[1,27]-s.l[1,21])
    trqlnk21_3_3 = -fSlnk23*(s.dpt[1,27]-s.l[1,21])
 
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
    frc[1,18] = s.frc[1,18]+frclnk18_3_1
    frc[2,18] = s.frc[2,18]+frclnk18_3_2
    frc[3,18] = s.frc[3,18]+frclnk18_3_3
    trq[1,18] = s.trq[1,18]+trqlnk18_3_1
    trq[2,18] = s.trq[2,18]+trqlnk18_3_2
    trq[3,18] = s.trq[3,18]+trqlnk18_3_3
    frc[1,19] = s.frc[1,19]-fSlnk12
    frc[2,19] = s.frc[2,19]-fSlnk22
    frc[3,19] = s.frc[3,19]-fSlnk32
    trq[1,19] = s.trq[1,19]+trqlnk19_2_1
    trq[2,19] = s.trq[2,19]+trqlnk19_2_2
    trq[3,19] = s.trq[3,19]+trqlnk19_2_3
    frc[1,21] = s.frc[1,21]-fSlnk13
    frc[2,21] = s.frc[2,21]-fSlnk23
    frc[3,21] = s.frc[3,21]-fSlnk33
    trq[1,21] = s.trq[1,21]+trqlnk21_3_1
    trq[2,21] = s.trq[2,21]+trqlnk21_3_2
    trq[3,21] = s.trq[3,21]+trqlnk21_3_3
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1
    Z[2] = Z2
    Zd[2] = Zd2
    Flink[2] = Flink2
    Z[3] = Z3
    Zd[3] = Zd3
    Flink[3] = Flink3

# Number of continuation lines = 0


