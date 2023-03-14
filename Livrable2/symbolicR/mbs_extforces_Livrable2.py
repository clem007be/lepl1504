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
#	==> Generation Date: Tue Mar 14 12:50:37 2023
#
#	==> Project name: Livrable2
#
#	==> Number of joints: 10
#
#	==> Function: F19 - External Forces
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos, sqrt
from numpy import zeros

def extforces(frc, trq, s, tsim):
    q = s.q
    qd = s.qd
    qdd = s.qdd
    frc = s.frc
    trq = s.trq
    PxF1 = zeros(4)
    RxF1 = zeros((4, 4))
    VxF1 = zeros(4)
    OMxF1 = zeros(4)
    AxF1 = zeros(4)
    OMPxF1 = zeros(4)

    PxF2 = zeros(4)
    RxF2 = zeros((4, 4))
    VxF2 = zeros(4)
    OMxF2 = zeros(4)
    AxF2 = zeros(4)
    OMPxF2 = zeros(4)

 
# Trigonometric functions

    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
    S6 = sin(q[6])
    C6 = cos(q[6])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics

    ROcp1_45 = -S4*C5
    ROcp1_55 = C4*C5
    ROcp1_75 = S4*S5
    ROcp1_85 = -C4*S5
    ROcp1_16 = -ROcp1_75*S6+C4*C6
    ROcp1_26 = -ROcp1_85*S6+S4*C6
    ROcp1_36 = -C5*S6
    ROcp1_76 = ROcp1_75*C6+C4*S6
    ROcp1_86 = ROcp1_85*C6+S4*S6
    ROcp1_96 = C5*C6
    ROcp1_17 = ROcp1_16*C7-ROcp1_76*S7
    ROcp1_27 = ROcp1_26*C7-ROcp1_86*S7
    ROcp1_37 = ROcp1_36*C7-ROcp1_96*S7
    ROcp1_77 = ROcp1_16*S7+ROcp1_76*C7
    ROcp1_87 = ROcp1_26*S7+ROcp1_86*C7
    ROcp1_97 = ROcp1_36*S7+ROcp1_96*C7
    OMcp1_15 = qd[5]*C4
    OMcp1_25 = qd[5]*S4
    OPcp1_15 = -qd[4]*qd[5]*S4+qdd[5]*C4
    OPcp1_25 = qd[4]*qd[5]*C4+qdd[5]*S4
    OMcp1_16 = OMcp1_15+qd[6]*ROcp1_45
    OMcp1_26 = OMcp1_25+qd[6]*ROcp1_55
    OMcp1_36 = qd[4]+qd[6]*S5
    OPcp1_16 = OPcp1_15+qd[6]*(-qd[4]*ROcp1_55+OMcp1_25*S5)+qdd[6]*ROcp1_45
    OPcp1_26 = OPcp1_25+qd[6]*(qd[4]*ROcp1_45-OMcp1_15*S5)+qdd[6]*ROcp1_55
    OPcp1_36 = qdd[4]+qd[6]*(OMcp1_15*ROcp1_55-OMcp1_25*ROcp1_45)+qdd[6]*S5
    RLcp1_17 = ROcp1_16*s.dpt[1,1]
    RLcp1_27 = ROcp1_26*s.dpt[1,1]
    RLcp1_37 = ROcp1_36*s.dpt[1,1]
    POcp1_17 = q[1]+RLcp1_17
    POcp1_27 = q[2]+RLcp1_27
    POcp1_37 = q[3]+RLcp1_37
    OMcp1_17 = OMcp1_16+qd[7]*ROcp1_45
    OMcp1_27 = OMcp1_26+qd[7]*ROcp1_55
    OMcp1_37 = OMcp1_36+qd[7]*S5
    ORcp1_17 = OMcp1_26*RLcp1_37-OMcp1_36*RLcp1_27
    ORcp1_27 = -OMcp1_16*RLcp1_37+OMcp1_36*RLcp1_17
    ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17
    VIcp1_17 = qd[1]+ORcp1_17
    VIcp1_27 = qd[2]+ORcp1_27
    VIcp1_37 = qd[3]+ORcp1_37
    OPcp1_17 = OPcp1_16+qd[7]*(OMcp1_26*S5-OMcp1_36*ROcp1_55)+qdd[7]*ROcp1_45
    OPcp1_27 = OPcp1_26+qd[7]*(-OMcp1_16*S5+OMcp1_36*ROcp1_45)+qdd[7]*ROcp1_55
    OPcp1_37 = OPcp1_36+qd[7]*(OMcp1_16*ROcp1_55-OMcp1_26*ROcp1_45)+qdd[7]*S5
    ACcp1_17 = qdd[1]+OMcp1_26*ORcp1_37-OMcp1_36*ORcp1_27+OPcp1_26*RLcp1_37-OPcp1_36*RLcp1_27
    ACcp1_27 = qdd[2]-OMcp1_16*ORcp1_37+OMcp1_36*ORcp1_17-OPcp1_16*RLcp1_37+OPcp1_36*RLcp1_17
    ACcp1_37 = qdd[3]+OMcp1_16*ORcp1_27-OMcp1_26*ORcp1_17+OPcp1_16*RLcp1_27-OPcp1_26*RLcp1_17
    PxF1[1] = POcp1_17
    PxF1[2] = POcp1_27
    PxF1[3] = POcp1_37
    RxF1[1,1] = ROcp1_17
    RxF1[1,2] = ROcp1_27
    RxF1[1,3] = ROcp1_37
    RxF1[2,1] = ROcp1_45
    RxF1[2,2] = ROcp1_55
    RxF1[2,3] = S5
    RxF1[3,1] = ROcp1_77
    RxF1[3,2] = ROcp1_87
    RxF1[3,3] = ROcp1_97
    VxF1[1] = VIcp1_17
    VxF1[2] = VIcp1_27
    VxF1[3] = VIcp1_37
    OMxF1[1] = OMcp1_17
    OMxF1[2] = OMcp1_27
    OMxF1[3] = OMcp1_37
    AxF1[1] = ACcp1_17
    AxF1[2] = ACcp1_27
    AxF1[3] = ACcp1_37
    OMPxF1[1] = OPcp1_17
    OMPxF1[2] = OPcp1_27
    OMPxF1[3] = OPcp1_37
    ROcp2_45 = -S4*C5
    ROcp2_55 = C4*C5
    ROcp2_75 = S4*S5
    ROcp2_85 = -C4*S5
    ROcp2_16 = -ROcp2_75*S6+C4*C6
    ROcp2_26 = -ROcp2_85*S6+S4*C6
    ROcp2_36 = -C5*S6
    ROcp2_76 = ROcp2_75*C6+C4*S6
    ROcp2_86 = ROcp2_85*C6+S4*S6
    ROcp2_96 = C5*C6
    ROcp2_18 = ROcp2_16*C8-ROcp2_76*S8
    ROcp2_28 = ROcp2_26*C8-ROcp2_86*S8
    ROcp2_38 = ROcp2_36*C8-ROcp2_96*S8
    ROcp2_78 = ROcp2_16*S8+ROcp2_76*C8
    ROcp2_88 = ROcp2_26*S8+ROcp2_86*C8
    ROcp2_98 = ROcp2_36*S8+ROcp2_96*C8
    ROcp2_19 = ROcp2_18*C9+ROcp2_45*S9
    ROcp2_29 = ROcp2_28*C9+ROcp2_55*S9
    ROcp2_39 = ROcp2_38*C9+S5*S9
    ROcp2_49 = -ROcp2_18*S9+ROcp2_45*C9
    ROcp2_59 = -ROcp2_28*S9+ROcp2_55*C9
    ROcp2_69 = -ROcp2_38*S9+S5*C9
    ROcp2_110 = ROcp2_19*C10-ROcp2_78*S10
    ROcp2_210 = ROcp2_29*C10-ROcp2_88*S10
    ROcp2_310 = ROcp2_39*C10-ROcp2_98*S10
    ROcp2_710 = ROcp2_19*S10+ROcp2_78*C10
    ROcp2_810 = ROcp2_29*S10+ROcp2_88*C10
    ROcp2_910 = ROcp2_39*S10+ROcp2_98*C10
    OMcp2_15 = qd[5]*C4
    OMcp2_25 = qd[5]*S4
    OPcp2_15 = -qd[4]*qd[5]*S4+qdd[5]*C4
    OPcp2_25 = qd[4]*qd[5]*C4+qdd[5]*S4
    OMcp2_16 = OMcp2_15+qd[6]*ROcp2_45
    OMcp2_26 = OMcp2_25+qd[6]*ROcp2_55
    OMcp2_36 = qd[4]+qd[6]*S5
    OPcp2_16 = OPcp2_15+qd[6]*(-qd[4]*ROcp2_55+OMcp2_25*S5)+qdd[6]*ROcp2_45
    OPcp2_26 = OPcp2_25+qd[6]*(qd[4]*ROcp2_45-OMcp2_15*S5)+qdd[6]*ROcp2_55
    OPcp2_36 = qdd[4]+qd[6]*(OMcp2_15*ROcp2_55-OMcp2_25*ROcp2_45)+qdd[6]*S5
    RLcp2_17 = ROcp2_16*s.dpt[1,2]+ROcp2_76*s.dpt[3,2]
    RLcp2_27 = ROcp2_26*s.dpt[1,2]+ROcp2_86*s.dpt[3,2]
    RLcp2_37 = ROcp2_36*s.dpt[1,2]+ROcp2_96*s.dpt[3,2]
    POcp2_17 = q[1]+RLcp2_17
    POcp2_27 = q[2]+RLcp2_27
    POcp2_37 = q[3]+RLcp2_37
    OMcp2_17 = OMcp2_16+qd[8]*ROcp2_45
    OMcp2_27 = OMcp2_26+qd[8]*ROcp2_55
    OMcp2_37 = OMcp2_36+qd[8]*S5
    ORcp2_17 = OMcp2_26*RLcp2_37-OMcp2_36*RLcp2_27
    ORcp2_27 = -OMcp2_16*RLcp2_37+OMcp2_36*RLcp2_17
    ORcp2_37 = OMcp2_16*RLcp2_27-OMcp2_26*RLcp2_17
    VIcp2_17 = qd[1]+ORcp2_17
    VIcp2_27 = qd[2]+ORcp2_27
    VIcp2_37 = qd[3]+ORcp2_37
    OPcp2_17 = OPcp2_16+qd[8]*(OMcp2_26*S5-OMcp2_36*ROcp2_55)+qdd[8]*ROcp2_45
    OPcp2_27 = OPcp2_26+qd[8]*(-OMcp2_16*S5+OMcp2_36*ROcp2_45)+qdd[8]*ROcp2_55
    OPcp2_37 = OPcp2_36+qd[8]*(OMcp2_16*ROcp2_55-OMcp2_26*ROcp2_45)+qdd[8]*S5
    ACcp2_17 = qdd[1]+OMcp2_26*ORcp2_37-OMcp2_36*ORcp2_27+OPcp2_26*RLcp2_37-OPcp2_36*RLcp2_27
    ACcp2_27 = qdd[2]-OMcp2_16*ORcp2_37+OMcp2_36*ORcp2_17-OPcp2_16*RLcp2_37+OPcp2_36*RLcp2_17
    ACcp2_37 = qdd[3]+OMcp2_16*ORcp2_27-OMcp2_26*ORcp2_17+OPcp2_16*RLcp2_27-OPcp2_26*RLcp2_17
    OMcp2_18 = OMcp2_17+qd[9]*ROcp2_78
    OMcp2_28 = OMcp2_27+qd[9]*ROcp2_88
    OMcp2_38 = OMcp2_37+qd[9]*ROcp2_98
    OPcp2_18 = OPcp2_17+qd[9]*(OMcp2_27*ROcp2_98-OMcp2_37*ROcp2_88)+qdd[9]*ROcp2_78
    OPcp2_28 = OPcp2_27+qd[9]*(-OMcp2_17*ROcp2_98+OMcp2_37*ROcp2_78)+qdd[9]*ROcp2_88
    OPcp2_38 = OPcp2_37+qd[9]*(OMcp2_17*ROcp2_88-OMcp2_27*ROcp2_78)+qdd[9]*ROcp2_98
    RLcp2_19 = ROcp2_78*s.dpt[3,4]
    RLcp2_29 = ROcp2_88*s.dpt[3,4]
    RLcp2_39 = ROcp2_98*s.dpt[3,4]
    POcp2_19 = POcp2_17+RLcp2_19
    POcp2_29 = POcp2_27+RLcp2_29
    POcp2_39 = POcp2_37+RLcp2_39
    OMcp2_19 = OMcp2_18+qd[10]*ROcp2_49
    OMcp2_29 = OMcp2_28+qd[10]*ROcp2_59
    OMcp2_39 = OMcp2_38+qd[10]*ROcp2_69
    ORcp2_19 = OMcp2_28*RLcp2_39-OMcp2_38*RLcp2_29
    ORcp2_29 = -OMcp2_18*RLcp2_39+OMcp2_38*RLcp2_19
    ORcp2_39 = OMcp2_18*RLcp2_29-OMcp2_28*RLcp2_19
    VIcp2_19 = ORcp2_19+VIcp2_17
    VIcp2_29 = ORcp2_29+VIcp2_27
    VIcp2_39 = ORcp2_39+VIcp2_37
    OPcp2_19 = OPcp2_18+qd[10]*(OMcp2_28*ROcp2_69-OMcp2_38*ROcp2_59)+qdd[10]*ROcp2_49
    OPcp2_29 = OPcp2_28+qd[10]*(-OMcp2_18*ROcp2_69+OMcp2_38*ROcp2_49)+qdd[10]*ROcp2_59
    OPcp2_39 = OPcp2_38+qd[10]*(OMcp2_18*ROcp2_59-OMcp2_28*ROcp2_49)+qdd[10]*ROcp2_69
    ACcp2_19 = ACcp2_17+OMcp2_28*ORcp2_39-OMcp2_38*ORcp2_29+OPcp2_28*RLcp2_39-OPcp2_38*RLcp2_29
    ACcp2_29 = ACcp2_27-OMcp2_18*ORcp2_39+OMcp2_38*ORcp2_19-OPcp2_18*RLcp2_39+OPcp2_38*RLcp2_19
    ACcp2_39 = ACcp2_37+OMcp2_18*ORcp2_29-OMcp2_28*ORcp2_19+OPcp2_18*RLcp2_29-OPcp2_28*RLcp2_19
    PxF2[1] = POcp2_19
    PxF2[2] = POcp2_29
    PxF2[3] = POcp2_39
    RxF2[1,1] = ROcp2_110
    RxF2[1,2] = ROcp2_210
    RxF2[1,3] = ROcp2_310
    RxF2[2,1] = ROcp2_49
    RxF2[2,2] = ROcp2_59
    RxF2[2,3] = ROcp2_69
    RxF2[3,1] = ROcp2_710
    RxF2[3,2] = ROcp2_810
    RxF2[3,3] = ROcp2_910
    VxF2[1] = VIcp2_19
    VxF2[2] = VIcp2_29
    VxF2[3] = VIcp2_39
    OMxF2[1] = OMcp2_19
    OMxF2[2] = OMcp2_29
    OMxF2[3] = OMcp2_39
    AxF2[1] = ACcp2_19
    AxF2[2] = ACcp2_29
    AxF2[3] = ACcp2_39
    OMPxF2[1] = OPcp2_19
    OMPxF2[2] = OPcp2_29
    OMPxF2[3] = OPcp2_39
 
# Sensor Forces 

    SWr1 = s.user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1)
    SWr2 = s.user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2)
    xfrc11 = RxF1[1,1]*SWr1[1]+RxF1[1,2]*SWr1[2]+RxF1[1,3]*SWr1[3]
    xfrc21 = RxF1[2,1]*SWr1[1]+RxF1[2,2]*SWr1[2]+RxF1[2,3]*SWr1[3]
    xfrc31 = RxF1[3,1]*SWr1[1]+RxF1[3,2]*SWr1[2]+RxF1[3,3]*SWr1[3]
    xtrq11 = RxF1[1,1]*SWr1[4]+RxF1[1,2]*SWr1[5]+RxF1[1,3]*SWr1[6]
    xtrq21 = RxF1[2,1]*SWr1[4]+RxF1[2,2]*SWr1[5]+RxF1[2,3]*SWr1[6]
    xtrq31 = RxF1[3,1]*SWr1[4]+RxF1[3,2]*SWr1[5]+RxF1[3,3]*SWr1[6]
    trqext_1_7_0 = xtrq11-xfrc21*SWr1[9]+xfrc31*SWr1[8]
    trqext_2_7_0 = xtrq21+xfrc11*SWr1[9]-xfrc31*SWr1[7]
    trqext_3_7_0 = xtrq31-xfrc11*SWr1[8]+xfrc21*SWr1[7]
    xfrc12 = RxF2[1,1]*SWr2[1]+RxF2[1,2]*SWr2[2]+RxF2[1,3]*SWr2[3]
    xfrc22 = RxF2[2,1]*SWr2[1]+RxF2[2,2]*SWr2[2]+RxF2[2,3]*SWr2[3]
    xfrc32 = RxF2[3,1]*SWr2[1]+RxF2[3,2]*SWr2[2]+RxF2[3,3]*SWr2[3]
    xtrq12 = RxF2[1,1]*SWr2[4]+RxF2[1,2]*SWr2[5]+RxF2[1,3]*SWr2[6]
    xtrq22 = RxF2[2,1]*SWr2[4]+RxF2[2,2]*SWr2[5]+RxF2[2,3]*SWr2[6]
    xtrq32 = RxF2[3,1]*SWr2[4]+RxF2[3,2]*SWr2[5]+RxF2[3,3]*SWr2[6]
    trqext_1_10_1 = xtrq12-xfrc22*SWr2[9]+xfrc32*SWr2[8]
    trqext_2_10_1 = xtrq22+xfrc12*SWr2[9]-xfrc32*SWr2[7]
    trqext_3_10_1 = xtrq32-xfrc12*SWr2[8]+xfrc22*SWr2[7]
 
# Symbolic model output

    frc[1,7] = s.frc[1,7]+xfrc11
    frc[2,7] = s.frc[2,7]+xfrc21
    frc[3,7] = s.frc[3,7]+xfrc31
    trq[1,7] = s.trq[1,7]+trqext_1_7_0
    trq[2,7] = s.trq[2,7]+trqext_2_7_0
    trq[3,7] = s.trq[3,7]+trqext_3_7_0
    frc[1,10] = s.frc[1,10]+xfrc12
    frc[2,10] = s.frc[2,10]+xfrc22
    frc[3,10] = s.frc[3,10]+xfrc32
    trq[1,10] = s.trq[1,10]+trqext_1_10_1
    trq[2,10] = s.trq[2,10]+trqext_2_10_1
    trq[3,10] = s.trq[3,10]+trqext_3_10_1

# Number of continuation lines = 0


