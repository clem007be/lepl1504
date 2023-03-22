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

    PxF3 = zeros(4)
    RxF3 = zeros((4, 4))
    VxF3 = zeros(4)
    OMxF3 = zeros(4)
    AxF3 = zeros(4)
    OMPxF3 = zeros(4)

 
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

    ROcp3_45 = -S4*C5
    ROcp3_55 = C4*C5
    ROcp3_75 = S4*S5
    ROcp3_85 = -C4*S5
    ROcp3_16 = -ROcp3_75*S6+C4*C6
    ROcp3_26 = -ROcp3_85*S6+S4*C6
    ROcp3_36 = -C5*S6
    ROcp3_76 = ROcp3_75*C6+C4*S6
    ROcp3_86 = ROcp3_85*C6+S4*S6
    ROcp3_96 = C5*C6
    ROcp3_17 = ROcp3_16*C7-ROcp3_76*S7
    ROcp3_27 = ROcp3_26*C7-ROcp3_86*S7
    ROcp3_37 = ROcp3_36*C7-ROcp3_96*S7
    ROcp3_77 = ROcp3_16*S7+ROcp3_76*C7
    ROcp3_87 = ROcp3_26*S7+ROcp3_86*C7
    ROcp3_97 = ROcp3_36*S7+ROcp3_96*C7
    OMcp3_15 = qd[5]*C4
    OMcp3_25 = qd[5]*S4
    OPcp3_15 = -qd[4]*qd[5]*S4+qdd[5]*C4
    OPcp3_25 = qd[4]*qd[5]*C4+qdd[5]*S4
    OMcp3_16 = OMcp3_15+qd[6]*ROcp3_45
    OMcp3_26 = OMcp3_25+qd[6]*ROcp3_55
    OMcp3_36 = qd[4]+qd[6]*S5
    OPcp3_16 = OPcp3_15+qd[6]*(-qd[4]*ROcp3_55+OMcp3_25*S5)+qdd[6]*ROcp3_45
    OPcp3_26 = OPcp3_25+qd[6]*(qd[4]*ROcp3_45-OMcp3_15*S5)+qdd[6]*ROcp3_55
    OPcp3_36 = qdd[4]+qd[6]*(OMcp3_15*ROcp3_55-OMcp3_25*ROcp3_45)+qdd[6]*S5
    RLcp3_17 = ROcp3_16*s.dpt[1,2]
    RLcp3_27 = ROcp3_26*s.dpt[1,2]
    RLcp3_37 = ROcp3_36*s.dpt[1,2]
    POcp3_17 = q[1]+RLcp3_17
    POcp3_27 = q[2]+RLcp3_27
    POcp3_37 = q[3]+RLcp3_37
    OMcp3_17 = OMcp3_16+qd[7]*ROcp3_45
    OMcp3_27 = OMcp3_26+qd[7]*ROcp3_55
    OMcp3_37 = OMcp3_36+qd[7]*S5
    ORcp3_17 = OMcp3_26*RLcp3_37-OMcp3_36*RLcp3_27
    ORcp3_27 = -OMcp3_16*RLcp3_37+OMcp3_36*RLcp3_17
    ORcp3_37 = OMcp3_16*RLcp3_27-OMcp3_26*RLcp3_17
    VIcp3_17 = qd[1]+ORcp3_17
    VIcp3_27 = qd[2]+ORcp3_27
    VIcp3_37 = qd[3]+ORcp3_37
    OPcp3_17 = OPcp3_16+qd[7]*(OMcp3_26*S5-OMcp3_36*ROcp3_55)+qdd[7]*ROcp3_45
    OPcp3_27 = OPcp3_26+qd[7]*(-OMcp3_16*S5+OMcp3_36*ROcp3_45)+qdd[7]*ROcp3_55
    OPcp3_37 = OPcp3_36+qd[7]*(OMcp3_16*ROcp3_55-OMcp3_26*ROcp3_45)+qdd[7]*S5
    ACcp3_17 = qdd[1]+OMcp3_26*ORcp3_37-OMcp3_36*ORcp3_27+OPcp3_26*RLcp3_37-OPcp3_36*RLcp3_27
    ACcp3_27 = qdd[2]-OMcp3_16*ORcp3_37+OMcp3_36*ORcp3_17-OPcp3_16*RLcp3_37+OPcp3_36*RLcp3_17
    ACcp3_37 = qdd[3]+OMcp3_16*ORcp3_27-OMcp3_26*ORcp3_17+OPcp3_16*RLcp3_27-OPcp3_26*RLcp3_17
    PxF1[1] = POcp3_17
    PxF1[2] = POcp3_27
    PxF1[3] = POcp3_37
    RxF1[1,1] = ROcp3_17
    RxF1[1,2] = ROcp3_27
    RxF1[1,3] = ROcp3_37
    RxF1[2,1] = ROcp3_45
    RxF1[2,2] = ROcp3_55
    RxF1[2,3] = S5
    RxF1[3,1] = ROcp3_77
    RxF1[3,2] = ROcp3_87
    RxF1[3,3] = ROcp3_97
    VxF1[1] = VIcp3_17
    VxF1[2] = VIcp3_27
    VxF1[3] = VIcp3_37
    OMxF1[1] = OMcp3_17
    OMxF1[2] = OMcp3_27
    OMxF1[3] = OMcp3_37
    AxF1[1] = ACcp3_17
    AxF1[2] = ACcp3_27
    AxF1[3] = ACcp3_37
    OMPxF1[1] = OPcp3_17
    OMPxF1[2] = OPcp3_27
    OMPxF1[3] = OPcp3_37
    ROcp4_45 = -S4*C5
    ROcp4_55 = C4*C5
    ROcp4_75 = S4*S5
    ROcp4_85 = -C4*S5
    ROcp4_16 = -ROcp4_75*S6+C4*C6
    ROcp4_26 = -ROcp4_85*S6+S4*C6
    ROcp4_36 = -C5*S6
    ROcp4_76 = ROcp4_75*C6+C4*S6
    ROcp4_86 = ROcp4_85*C6+S4*S6
    ROcp4_96 = C5*C6
    ROcp4_18 = ROcp4_16*C8-ROcp4_76*S8
    ROcp4_28 = ROcp4_26*C8-ROcp4_86*S8
    ROcp4_38 = ROcp4_36*C8-ROcp4_96*S8
    ROcp4_78 = ROcp4_16*S8+ROcp4_76*C8
    ROcp4_88 = ROcp4_26*S8+ROcp4_86*C8
    ROcp4_98 = ROcp4_36*S8+ROcp4_96*C8
    ROcp4_19 = ROcp4_18*C9+ROcp4_45*S9
    ROcp4_29 = ROcp4_28*C9+ROcp4_55*S9
    ROcp4_39 = ROcp4_38*C9+S5*S9
    ROcp4_49 = -ROcp4_18*S9+ROcp4_45*C9
    ROcp4_59 = -ROcp4_28*S9+ROcp4_55*C9
    ROcp4_69 = -ROcp4_38*S9+S5*C9
    OMcp4_15 = qd[5]*C4
    OMcp4_25 = qd[5]*S4
    OPcp4_15 = -qd[4]*qd[5]*S4+qdd[5]*C4
    OPcp4_25 = qd[4]*qd[5]*C4+qdd[5]*S4
    OMcp4_16 = OMcp4_15+qd[6]*ROcp4_45
    OMcp4_26 = OMcp4_25+qd[6]*ROcp4_55
    OMcp4_36 = qd[4]+qd[6]*S5
    OPcp4_16 = OPcp4_15+qd[6]*(-qd[4]*ROcp4_55+OMcp4_25*S5)+qdd[6]*ROcp4_45
    OPcp4_26 = OPcp4_25+qd[6]*(qd[4]*ROcp4_45-OMcp4_15*S5)+qdd[6]*ROcp4_55
    OPcp4_36 = qdd[4]+qd[6]*(OMcp4_15*ROcp4_55-OMcp4_25*ROcp4_45)+qdd[6]*S5
    RLcp4_17 = ROcp4_16*s.dpt[1,3]+ROcp4_76*s.dpt[3,3]
    RLcp4_27 = ROcp4_26*s.dpt[1,3]+ROcp4_86*s.dpt[3,3]
    RLcp4_37 = ROcp4_36*s.dpt[1,3]+ROcp4_96*s.dpt[3,3]
    POcp4_17 = q[1]+RLcp4_17
    POcp4_27 = q[2]+RLcp4_27
    POcp4_37 = q[3]+RLcp4_37
    OMcp4_17 = OMcp4_16+qd[8]*ROcp4_45
    OMcp4_27 = OMcp4_26+qd[8]*ROcp4_55
    OMcp4_37 = OMcp4_36+qd[8]*S5
    ORcp4_17 = OMcp4_26*RLcp4_37-OMcp4_36*RLcp4_27
    ORcp4_27 = -OMcp4_16*RLcp4_37+OMcp4_36*RLcp4_17
    ORcp4_37 = OMcp4_16*RLcp4_27-OMcp4_26*RLcp4_17
    VIcp4_17 = qd[1]+ORcp4_17
    VIcp4_27 = qd[2]+ORcp4_27
    VIcp4_37 = qd[3]+ORcp4_37
    OPcp4_17 = OPcp4_16+qd[8]*(OMcp4_26*S5-OMcp4_36*ROcp4_55)+qdd[8]*ROcp4_45
    OPcp4_27 = OPcp4_26+qd[8]*(-OMcp4_16*S5+OMcp4_36*ROcp4_45)+qdd[8]*ROcp4_55
    OPcp4_37 = OPcp4_36+qd[8]*(OMcp4_16*ROcp4_55-OMcp4_26*ROcp4_45)+qdd[8]*S5
    ACcp4_17 = qdd[1]+OMcp4_26*ORcp4_37-OMcp4_36*ORcp4_27+OPcp4_26*RLcp4_37-OPcp4_36*RLcp4_27
    ACcp4_27 = qdd[2]-OMcp4_16*ORcp4_37+OMcp4_36*ORcp4_17-OPcp4_16*RLcp4_37+OPcp4_36*RLcp4_17
    ACcp4_37 = qdd[3]+OMcp4_16*ORcp4_27-OMcp4_26*ORcp4_17+OPcp4_16*RLcp4_27-OPcp4_26*RLcp4_17
    OMcp4_18 = OMcp4_17+qd[9]*ROcp4_78
    OMcp4_28 = OMcp4_27+qd[9]*ROcp4_88
    OMcp4_38 = OMcp4_37+qd[9]*ROcp4_98
    OPcp4_18 = OPcp4_17+qd[9]*(OMcp4_27*ROcp4_98-OMcp4_37*ROcp4_88)+qdd[9]*ROcp4_78
    OPcp4_28 = OPcp4_27+qd[9]*(-OMcp4_17*ROcp4_98+OMcp4_37*ROcp4_78)+qdd[9]*ROcp4_88
    OPcp4_38 = OPcp4_37+qd[9]*(OMcp4_17*ROcp4_88-OMcp4_27*ROcp4_78)+qdd[9]*ROcp4_98
    PxF2[1] = POcp4_17
    PxF2[2] = POcp4_27
    PxF2[3] = POcp4_37
    RxF2[1,1] = ROcp4_19
    RxF2[1,2] = ROcp4_29
    RxF2[1,3] = ROcp4_39
    RxF2[2,1] = ROcp4_49
    RxF2[2,2] = ROcp4_59
    RxF2[2,3] = ROcp4_69
    RxF2[3,1] = ROcp4_78
    RxF2[3,2] = ROcp4_88
    RxF2[3,3] = ROcp4_98
    VxF2[1] = VIcp4_17
    VxF2[2] = VIcp4_27
    VxF2[3] = VIcp4_37
    OMxF2[1] = OMcp4_18
    OMxF2[2] = OMcp4_28
    OMxF2[3] = OMcp4_38
    AxF2[1] = ACcp4_17
    AxF2[2] = ACcp4_27
    AxF2[3] = ACcp4_37
    OMPxF2[1] = OPcp4_18
    OMPxF2[2] = OPcp4_28
    OMPxF2[3] = OPcp4_38
    ROcp5_45 = -S4*C5
    ROcp5_55 = C4*C5
    ROcp5_75 = S4*S5
    ROcp5_85 = -C4*S5
    ROcp5_16 = -ROcp5_75*S6+C4*C6
    ROcp5_26 = -ROcp5_85*S6+S4*C6
    ROcp5_36 = -C5*S6
    ROcp5_76 = ROcp5_75*C6+C4*S6
    ROcp5_86 = ROcp5_85*C6+S4*S6
    ROcp5_96 = C5*C6
    ROcp5_18 = ROcp5_16*C8-ROcp5_76*S8
    ROcp5_28 = ROcp5_26*C8-ROcp5_86*S8
    ROcp5_38 = ROcp5_36*C8-ROcp5_96*S8
    ROcp5_78 = ROcp5_16*S8+ROcp5_76*C8
    ROcp5_88 = ROcp5_26*S8+ROcp5_86*C8
    ROcp5_98 = ROcp5_36*S8+ROcp5_96*C8
    ROcp5_19 = ROcp5_18*C9+ROcp5_45*S9
    ROcp5_29 = ROcp5_28*C9+ROcp5_55*S9
    ROcp5_39 = ROcp5_38*C9+S5*S9
    ROcp5_49 = -ROcp5_18*S9+ROcp5_45*C9
    ROcp5_59 = -ROcp5_28*S9+ROcp5_55*C9
    ROcp5_69 = -ROcp5_38*S9+S5*C9
    ROcp5_110 = ROcp5_19*C10-ROcp5_78*S10
    ROcp5_210 = ROcp5_29*C10-ROcp5_88*S10
    ROcp5_310 = ROcp5_39*C10-ROcp5_98*S10
    ROcp5_710 = ROcp5_19*S10+ROcp5_78*C10
    ROcp5_810 = ROcp5_29*S10+ROcp5_88*C10
    ROcp5_910 = ROcp5_39*S10+ROcp5_98*C10
    OMcp5_15 = qd[5]*C4
    OMcp5_25 = qd[5]*S4
    OPcp5_15 = -qd[4]*qd[5]*S4+qdd[5]*C4
    OPcp5_25 = qd[4]*qd[5]*C4+qdd[5]*S4
    OMcp5_16 = OMcp5_15+qd[6]*ROcp5_45
    OMcp5_26 = OMcp5_25+qd[6]*ROcp5_55
    OMcp5_36 = qd[4]+qd[6]*S5
    OPcp5_16 = OPcp5_15+qd[6]*(-qd[4]*ROcp5_55+OMcp5_25*S5)+qdd[6]*ROcp5_45
    OPcp5_26 = OPcp5_25+qd[6]*(qd[4]*ROcp5_45-OMcp5_15*S5)+qdd[6]*ROcp5_55
    OPcp5_36 = qdd[4]+qd[6]*(OMcp5_15*ROcp5_55-OMcp5_25*ROcp5_45)+qdd[6]*S5
    RLcp5_17 = ROcp5_16*s.dpt[1,3]+ROcp5_76*s.dpt[3,3]
    RLcp5_27 = ROcp5_26*s.dpt[1,3]+ROcp5_86*s.dpt[3,3]
    RLcp5_37 = ROcp5_36*s.dpt[1,3]+ROcp5_96*s.dpt[3,3]
    POcp5_17 = q[1]+RLcp5_17
    POcp5_27 = q[2]+RLcp5_27
    POcp5_37 = q[3]+RLcp5_37
    OMcp5_17 = OMcp5_16+qd[8]*ROcp5_45
    OMcp5_27 = OMcp5_26+qd[8]*ROcp5_55
    OMcp5_37 = OMcp5_36+qd[8]*S5
    ORcp5_17 = OMcp5_26*RLcp5_37-OMcp5_36*RLcp5_27
    ORcp5_27 = -OMcp5_16*RLcp5_37+OMcp5_36*RLcp5_17
    ORcp5_37 = OMcp5_16*RLcp5_27-OMcp5_26*RLcp5_17
    VIcp5_17 = qd[1]+ORcp5_17
    VIcp5_27 = qd[2]+ORcp5_27
    VIcp5_37 = qd[3]+ORcp5_37
    OPcp5_17 = OPcp5_16+qd[8]*(OMcp5_26*S5-OMcp5_36*ROcp5_55)+qdd[8]*ROcp5_45
    OPcp5_27 = OPcp5_26+qd[8]*(-OMcp5_16*S5+OMcp5_36*ROcp5_45)+qdd[8]*ROcp5_55
    OPcp5_37 = OPcp5_36+qd[8]*(OMcp5_16*ROcp5_55-OMcp5_26*ROcp5_45)+qdd[8]*S5
    ACcp5_17 = qdd[1]+OMcp5_26*ORcp5_37-OMcp5_36*ORcp5_27+OPcp5_26*RLcp5_37-OPcp5_36*RLcp5_27
    ACcp5_27 = qdd[2]-OMcp5_16*ORcp5_37+OMcp5_36*ORcp5_17-OPcp5_16*RLcp5_37+OPcp5_36*RLcp5_17
    ACcp5_37 = qdd[3]+OMcp5_16*ORcp5_27-OMcp5_26*ORcp5_17+OPcp5_16*RLcp5_27-OPcp5_26*RLcp5_17
    OMcp5_18 = OMcp5_17+qd[9]*ROcp5_78
    OMcp5_28 = OMcp5_27+qd[9]*ROcp5_88
    OMcp5_38 = OMcp5_37+qd[9]*ROcp5_98
    OPcp5_18 = OPcp5_17+qd[9]*(OMcp5_27*ROcp5_98-OMcp5_37*ROcp5_88)+qdd[9]*ROcp5_78
    OPcp5_28 = OPcp5_27+qd[9]*(-OMcp5_17*ROcp5_98+OMcp5_37*ROcp5_78)+qdd[9]*ROcp5_88
    OPcp5_38 = OPcp5_37+qd[9]*(OMcp5_17*ROcp5_88-OMcp5_27*ROcp5_78)+qdd[9]*ROcp5_98
    RLcp5_19 = ROcp5_78*s.dpt[3,7]
    RLcp5_29 = ROcp5_88*s.dpt[3,7]
    RLcp5_39 = ROcp5_98*s.dpt[3,7]
    POcp5_19 = POcp5_17+RLcp5_19
    POcp5_29 = POcp5_27+RLcp5_29
    POcp5_39 = POcp5_37+RLcp5_39
    OMcp5_19 = OMcp5_18+qd[10]*ROcp5_49
    OMcp5_29 = OMcp5_28+qd[10]*ROcp5_59
    OMcp5_39 = OMcp5_38+qd[10]*ROcp5_69
    ORcp5_19 = OMcp5_28*RLcp5_39-OMcp5_38*RLcp5_29
    ORcp5_29 = -OMcp5_18*RLcp5_39+OMcp5_38*RLcp5_19
    ORcp5_39 = OMcp5_18*RLcp5_29-OMcp5_28*RLcp5_19
    VIcp5_19 = ORcp5_19+VIcp5_17
    VIcp5_29 = ORcp5_29+VIcp5_27
    VIcp5_39 = ORcp5_39+VIcp5_37
    OPcp5_19 = OPcp5_18+qd[10]*(OMcp5_28*ROcp5_69-OMcp5_38*ROcp5_59)+qdd[10]*ROcp5_49
    OPcp5_29 = OPcp5_28+qd[10]*(-OMcp5_18*ROcp5_69+OMcp5_38*ROcp5_49)+qdd[10]*ROcp5_59
    OPcp5_39 = OPcp5_38+qd[10]*(OMcp5_18*ROcp5_59-OMcp5_28*ROcp5_49)+qdd[10]*ROcp5_69
    ACcp5_19 = ACcp5_17+OMcp5_28*ORcp5_39-OMcp5_38*ORcp5_29+OPcp5_28*RLcp5_39-OPcp5_38*RLcp5_29
    ACcp5_29 = ACcp5_27-OMcp5_18*ORcp5_39+OMcp5_38*ORcp5_19-OPcp5_18*RLcp5_39+OPcp5_38*RLcp5_19
    ACcp5_39 = ACcp5_37+OMcp5_18*ORcp5_29-OMcp5_28*ORcp5_19+OPcp5_18*RLcp5_29-OPcp5_28*RLcp5_19
    PxF3[1] = POcp5_19
    PxF3[2] = POcp5_29
    PxF3[3] = POcp5_39
    RxF3[1,1] = ROcp5_110
    RxF3[1,2] = ROcp5_210
    RxF3[1,3] = ROcp5_310
    RxF3[2,1] = ROcp5_49
    RxF3[2,2] = ROcp5_59
    RxF3[2,3] = ROcp5_69
    RxF3[3,1] = ROcp5_710
    RxF3[3,2] = ROcp5_810
    RxF3[3,3] = ROcp5_910
    VxF3[1] = VIcp5_19
    VxF3[2] = VIcp5_29
    VxF3[3] = VIcp5_39
    OMxF3[1] = OMcp5_19
    OMxF3[2] = OMcp5_29
    OMxF3[3] = OMcp5_39
    AxF3[1] = ACcp5_19
    AxF3[2] = ACcp5_29
    AxF3[3] = ACcp5_39
    OMPxF3[1] = OPcp5_19
    OMPxF3[2] = OPcp5_29
    OMPxF3[3] = OPcp5_39
 
# Sensor Forces 

    SWr1 = s.user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1)
    SWr2 = s.user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2)
    SWr3 = s.user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3)
    xfrc13 = RxF1[1,1]*SWr1[1]+RxF1[1,2]*SWr1[2]+RxF1[1,3]*SWr1[3]
    xfrc23 = RxF1[2,1]*SWr1[1]+RxF1[2,2]*SWr1[2]+RxF1[2,3]*SWr1[3]
    xfrc33 = RxF1[3,1]*SWr1[1]+RxF1[3,2]*SWr1[2]+RxF1[3,3]*SWr1[3]
    xtrq13 = RxF1[1,1]*SWr1[4]+RxF1[1,2]*SWr1[5]+RxF1[1,3]*SWr1[6]
    xtrq23 = RxF1[2,1]*SWr1[4]+RxF1[2,2]*SWr1[5]+RxF1[2,3]*SWr1[6]
    xtrq33 = RxF1[3,1]*SWr1[4]+RxF1[3,2]*SWr1[5]+RxF1[3,3]*SWr1[6]
    trqext_1_7_2 = xtrq13-xfrc23*SWr1[9]+xfrc33*SWr1[8]
    trqext_2_7_2 = xtrq23+xfrc13*SWr1[9]-xfrc33*SWr1[7]
    trqext_3_7_2 = xtrq33-xfrc13*SWr1[8]+xfrc23*SWr1[7]
    xfrc14 = RxF2[1,1]*SWr2[1]+RxF2[1,2]*SWr2[2]+RxF2[1,3]*SWr2[3]
    xfrc24 = RxF2[2,1]*SWr2[1]+RxF2[2,2]*SWr2[2]+RxF2[2,3]*SWr2[3]
    xfrc34 = RxF2[3,1]*SWr2[1]+RxF2[3,2]*SWr2[2]+RxF2[3,3]*SWr2[3]
    xtrq14 = RxF2[1,1]*SWr2[4]+RxF2[1,2]*SWr2[5]+RxF2[1,3]*SWr2[6]
    xtrq24 = RxF2[2,1]*SWr2[4]+RxF2[2,2]*SWr2[5]+RxF2[2,3]*SWr2[6]
    xtrq34 = RxF2[3,1]*SWr2[4]+RxF2[3,2]*SWr2[5]+RxF2[3,3]*SWr2[6]
    trqext_1_9_3 = xtrq14-xfrc24*(SWr2[9]-s.l[3,9])+xfrc34*SWr2[8]
    trqext_2_9_3 = xtrq24+xfrc14*(SWr2[9]-s.l[3,9])-xfrc34*SWr2[7]
    trqext_3_9_3 = xtrq34-xfrc14*SWr2[8]+xfrc24*SWr2[7]
    xfrc15 = RxF3[1,1]*SWr3[1]+RxF3[1,2]*SWr3[2]+RxF3[1,3]*SWr3[3]
    xfrc25 = RxF3[2,1]*SWr3[1]+RxF3[2,2]*SWr3[2]+RxF3[2,3]*SWr3[3]
    xfrc35 = RxF3[3,1]*SWr3[1]+RxF3[3,2]*SWr3[2]+RxF3[3,3]*SWr3[3]
    xtrq15 = RxF3[1,1]*SWr3[4]+RxF3[1,2]*SWr3[5]+RxF3[1,3]*SWr3[6]
    xtrq25 = RxF3[2,1]*SWr3[4]+RxF3[2,2]*SWr3[5]+RxF3[2,3]*SWr3[6]
    xtrq35 = RxF3[3,1]*SWr3[4]+RxF3[3,2]*SWr3[5]+RxF3[3,3]*SWr3[6]
    trqext_1_10_4 = xtrq15-xfrc25*SWr3[9]+xfrc35*SWr3[8]
    trqext_2_10_4 = xtrq25+xfrc15*SWr3[9]-xfrc35*SWr3[7]
    trqext_3_10_4 = xtrq35-xfrc15*SWr3[8]+xfrc25*SWr3[7]
 
# Symbolic model output

    frc[1,7] = s.frc[1,7]+xfrc13
    frc[2,7] = s.frc[2,7]+xfrc23
    frc[3,7] = s.frc[3,7]+xfrc33
    trq[1,7] = s.trq[1,7]+trqext_1_7_2
    trq[2,7] = s.trq[2,7]+trqext_2_7_2
    trq[3,7] = s.trq[3,7]+trqext_3_7_2
    frc[1,9] = s.frc[1,9]+xfrc14
    frc[2,9] = s.frc[2,9]+xfrc24
    frc[3,9] = s.frc[3,9]+xfrc34
    trq[1,9] = s.trq[1,9]+trqext_1_9_3
    trq[2,9] = s.trq[2,9]+trqext_2_9_3
    trq[3,9] = s.trq[3,9]+trqext_3_9_3
    frc[1,10] = s.frc[1,10]+xfrc15
    frc[2,10] = s.frc[2,10]+xfrc25
    frc[3,10] = s.frc[3,10]+xfrc35
    trq[1,10] = s.trq[1,10]+trqext_1_10_4
    trq[2,10] = s.trq[2,10]+trqext_2_10_4
    trq[3,10] = s.trq[3,10]+trqext_3_10_4

# Number of continuation lines = 0


