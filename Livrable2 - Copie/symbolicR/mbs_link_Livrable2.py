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
#	==> Generation Date: Fri Mar 24 14:16:07 2023
#
#	==> Project name: Livrable2
#
#	==> Number of joints: 23
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
    S17 = sin(q[17])
    C17 = cos(q[17])
    S19 = sin(q[19])
    C19 = cos(q[19])
 
# Augmented Joint Position Vectors

    Dz223 = q[22]+s.dpt[3,14]
 
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
    RLlnk4_12 = s.dpt[1,22]*C17+s.dpt[3,22]*S17
    RLlnk4_32 = -s.dpt[1,22]*S17+s.dpt[3,22]*C17
    POlnk4_12 = RLlnk4_12+s.dpt[1,16]
    POlnk4_32 = RLlnk4_32+s.dpt[3,16]
    ORlnk4_12 = qd[17]*RLlnk4_32
    ORlnk4_32 = -qd[17]*RLlnk4_12
    Plnk12 = POlnk4_12-s.dpt[1,18]
    Plnk22 = s.dpt[2,16]-s.dpt[2,18]
    Plnk32 = POlnk4_32-s.dpt[3,18]
    PPlnk2 = Plnk12*Plnk12+Plnk22*Plnk22+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e12 = Plnk12/Z2
    e22 = Plnk22/Z2
    e32 = Plnk32/Z2
    Zd2 = ORlnk4_12*e12+ORlnk4_32*e32
    RLlnk6_12 = s.dpt[1,25]*C19+s.dpt[3,25]*S19
    RLlnk6_32 = -s.dpt[1,25]*S19+s.dpt[3,25]*C19
    POlnk6_12 = RLlnk6_12+s.dpt[1,17]
    POlnk6_32 = RLlnk6_32+s.dpt[3,17]
    ORlnk6_12 = qd[19]*RLlnk6_32
    ORlnk6_32 = -qd[19]*RLlnk6_12
    Plnk13 = POlnk6_12-s.dpt[1,19]
    Plnk23 = s.dpt[2,17]-s.dpt[2,19]
    Plnk33 = POlnk6_32-s.dpt[3,19]
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
    trqlnk16_2_1 = -fPlnk22*s.dpt[3,18]+fPlnk32*s.dpt[2,18]
    trqlnk16_2_2 = fPlnk12*s.dpt[3,18]-fPlnk32*(s.dpt[1,18]-s.l[1,16])
    trqlnk16_2_3 = -fPlnk12*s.dpt[2,18]+fPlnk22*(s.dpt[1,18]-s.l[1,16])
    fSlnk12 = Flink2*(e12*C17-e32*S17)
    fSlnk22 = Flink2*e22
    fSlnk32 = Flink2*(e12*S17+e32*C17)
    trqlnk17_2_1 = fSlnk22*s.dpt[3,22]
    trqlnk17_2_2 = -fSlnk12*s.dpt[3,22]+fSlnk32*(s.dpt[1,22]-s.l[1,17])
    trqlnk17_2_3 = -fSlnk22*(s.dpt[1,22]-s.l[1,17])
    fPlnk13 = Flink3*e13
    fPlnk23 = Flink3*e23
    fPlnk33 = Flink3*e33
    frclnk16_3_1 = fPlnk12+fPlnk13
    frclnk16_3_2 = fPlnk22+fPlnk23
    frclnk16_3_3 = fPlnk32+fPlnk33
    trqlnk16_3_1 = trqlnk16_2_1-fPlnk23*s.dpt[3,19]+fPlnk33*s.dpt[2,19]
    trqlnk16_3_2 = trqlnk16_2_2+fPlnk13*s.dpt[3,19]-fPlnk33*(s.dpt[1,19]-s.l[1,16])
    trqlnk16_3_3 = trqlnk16_2_3-fPlnk13*s.dpt[2,19]+fPlnk23*(s.dpt[1,19]-s.l[1,16])
    fSlnk13 = Flink3*(e13*C19-e33*S19)
    fSlnk23 = Flink3*e23
    fSlnk33 = Flink3*(e13*S19+e33*C19)
    trqlnk19_3_1 = fSlnk23*s.dpt[3,25]
    trqlnk19_3_2 = -fSlnk13*s.dpt[3,25]+fSlnk33*(s.dpt[1,25]-s.l[1,19])
    trqlnk19_3_3 = -fSlnk23*(s.dpt[1,25]-s.l[1,19])
 
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
    frc[1,16] = s.frc[1,16]+frclnk16_3_1
    frc[2,16] = s.frc[2,16]+frclnk16_3_2
    frc[3,16] = s.frc[3,16]+frclnk16_3_3
    trq[1,16] = s.trq[1,16]+trqlnk16_3_1
    trq[2,16] = s.trq[2,16]+trqlnk16_3_2
    trq[3,16] = s.trq[3,16]+trqlnk16_3_3
    frc[1,17] = s.frc[1,17]-fSlnk12
    frc[2,17] = s.frc[2,17]-fSlnk22
    frc[3,17] = s.frc[3,17]-fSlnk32
    trq[1,17] = s.trq[1,17]+trqlnk17_2_1
    trq[2,17] = s.trq[2,17]+trqlnk17_2_2
    trq[3,17] = s.trq[3,17]+trqlnk17_2_3
    frc[1,19] = s.frc[1,19]-fSlnk13
    frc[2,19] = s.frc[2,19]-fSlnk23
    frc[3,19] = s.frc[3,19]-fSlnk33
    trq[1,19] = s.trq[1,19]+trqlnk19_3_1
    trq[2,19] = s.trq[2,19]+trqlnk19_3_2
    trq[3,19] = s.trq[3,19]+trqlnk19_3_3
 
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


