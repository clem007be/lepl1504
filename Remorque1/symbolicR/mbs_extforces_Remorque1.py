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
    S11 = sin(q[11])
    C11 = cos(q[11])
    S12 = sin(q[12])
    C12 = cos(q[12])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S16 = sin(q[16])
    C16 = cos(q[16])
 
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
    ROcp1_18 = ROcp1_17*C8-ROcp1_77*S8
    ROcp1_28 = ROcp1_27*C8-ROcp1_87*S8
    ROcp1_38 = ROcp1_37*C8-ROcp1_97*S8
    ROcp1_78 = ROcp1_17*S8+ROcp1_77*C8
    ROcp1_88 = ROcp1_27*S8+ROcp1_87*C8
    ROcp1_98 = ROcp1_37*S8+ROcp1_97*C8
    ROcp1_19 = ROcp1_18*C9-ROcp1_78*S9
    ROcp1_29 = ROcp1_28*C9-ROcp1_88*S9
    ROcp1_39 = ROcp1_38*C9-ROcp1_98*S9
    ROcp1_79 = ROcp1_18*S9+ROcp1_78*C9
    ROcp1_89 = ROcp1_28*S9+ROcp1_88*C9
    ROcp1_99 = ROcp1_38*S9+ROcp1_98*C9
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
    RLcp1_17 = ROcp1_16*s.dpt[1,1]+ROcp1_76*s.dpt[3,1]
    RLcp1_27 = ROcp1_26*s.dpt[1,1]+ROcp1_86*s.dpt[3,1]
    RLcp1_37 = ROcp1_36*s.dpt[1,1]+ROcp1_96*s.dpt[3,1]
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
    RLcp1_18 = ROcp1_17*s.dpt[1,2]
    RLcp1_28 = ROcp1_27*s.dpt[1,2]
    RLcp1_38 = ROcp1_37*s.dpt[1,2]
    POcp1_18 = POcp1_17+RLcp1_18
    POcp1_28 = POcp1_27+RLcp1_28
    POcp1_38 = POcp1_37+RLcp1_38
    OMcp1_18 = OMcp1_17+qd[8]*ROcp1_45
    OMcp1_28 = OMcp1_27+qd[8]*ROcp1_55
    OMcp1_38 = OMcp1_37+qd[8]*S5
    ORcp1_18 = OMcp1_27*RLcp1_38-OMcp1_37*RLcp1_28
    ORcp1_28 = -OMcp1_17*RLcp1_38+OMcp1_37*RLcp1_18
    ORcp1_38 = OMcp1_17*RLcp1_28-OMcp1_27*RLcp1_18
    VIcp1_18 = ORcp1_18+VIcp1_17
    VIcp1_28 = ORcp1_28+VIcp1_27
    VIcp1_38 = ORcp1_38+VIcp1_37
    OPcp1_18 = OPcp1_17+qd[8]*(OMcp1_27*S5-OMcp1_37*ROcp1_55)+qdd[8]*ROcp1_45
    OPcp1_28 = OPcp1_27+qd[8]*(-OMcp1_17*S5+OMcp1_37*ROcp1_45)+qdd[8]*ROcp1_55
    OPcp1_38 = OPcp1_37+qd[8]*(OMcp1_17*ROcp1_55-OMcp1_27*ROcp1_45)+qdd[8]*S5
    ACcp1_18 = ACcp1_17+OMcp1_27*ORcp1_38-OMcp1_37*ORcp1_28+OPcp1_27*RLcp1_38-OPcp1_37*RLcp1_28
    ACcp1_28 = ACcp1_27-OMcp1_17*ORcp1_38+OMcp1_37*ORcp1_18-OPcp1_17*RLcp1_38+OPcp1_37*RLcp1_18
    ACcp1_38 = ACcp1_37+OMcp1_17*ORcp1_28-OMcp1_27*ORcp1_18+OPcp1_17*RLcp1_28-OPcp1_27*RLcp1_18
    RLcp1_19 = ROcp1_78*s.dpt[3,3]
    RLcp1_29 = ROcp1_88*s.dpt[3,3]
    RLcp1_39 = ROcp1_98*s.dpt[3,3]
    POcp1_19 = POcp1_18+RLcp1_19
    POcp1_29 = POcp1_28+RLcp1_29
    POcp1_39 = POcp1_38+RLcp1_39
    OMcp1_19 = OMcp1_18+qd[9]*ROcp1_45
    OMcp1_29 = OMcp1_28+qd[9]*ROcp1_55
    OMcp1_39 = OMcp1_38+qd[9]*S5
    ORcp1_19 = OMcp1_28*RLcp1_39-OMcp1_38*RLcp1_29
    ORcp1_29 = -OMcp1_18*RLcp1_39+OMcp1_38*RLcp1_19
    ORcp1_39 = OMcp1_18*RLcp1_29-OMcp1_28*RLcp1_19
    VIcp1_19 = ORcp1_19+VIcp1_18
    VIcp1_29 = ORcp1_29+VIcp1_28
    VIcp1_39 = ORcp1_39+VIcp1_38
    OPcp1_19 = OPcp1_18+qd[9]*(OMcp1_28*S5-OMcp1_38*ROcp1_55)+qdd[9]*ROcp1_45
    OPcp1_29 = OPcp1_28+qd[9]*(-OMcp1_18*S5+OMcp1_38*ROcp1_45)+qdd[9]*ROcp1_55
    OPcp1_39 = OPcp1_38+qd[9]*(OMcp1_18*ROcp1_55-OMcp1_28*ROcp1_45)+qdd[9]*S5
    ACcp1_19 = ACcp1_18+OMcp1_28*ORcp1_39-OMcp1_38*ORcp1_29+OPcp1_28*RLcp1_39-OPcp1_38*RLcp1_29
    ACcp1_29 = ACcp1_28-OMcp1_18*ORcp1_39+OMcp1_38*ORcp1_19-OPcp1_18*RLcp1_39+OPcp1_38*RLcp1_19
    ACcp1_39 = ACcp1_38+OMcp1_18*ORcp1_29-OMcp1_28*ORcp1_19+OPcp1_18*RLcp1_29-OPcp1_28*RLcp1_19
    PxF1[1] = POcp1_19
    PxF1[2] = POcp1_29
    PxF1[3] = POcp1_39
    RxF1[1,1] = ROcp1_19
    RxF1[1,2] = ROcp1_29
    RxF1[1,3] = ROcp1_39
    RxF1[2,1] = ROcp1_45
    RxF1[2,2] = ROcp1_55
    RxF1[2,3] = S5
    RxF1[3,1] = ROcp1_79
    RxF1[3,2] = ROcp1_89
    RxF1[3,3] = ROcp1_99
    VxF1[1] = VIcp1_19
    VxF1[2] = VIcp1_29
    VxF1[3] = VIcp1_39
    OMxF1[1] = OMcp1_19
    OMxF1[2] = OMcp1_29
    OMxF1[3] = OMcp1_39
    AxF1[1] = ACcp1_19
    AxF1[2] = ACcp1_29
    AxF1[3] = ACcp1_39
    OMPxF1[1] = OPcp1_19
    OMPxF1[2] = OPcp1_29
    OMPxF1[3] = OPcp1_39
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
    ROcp2_17 = ROcp2_16*C7-ROcp2_76*S7
    ROcp2_27 = ROcp2_26*C7-ROcp2_86*S7
    ROcp2_37 = ROcp2_36*C7-ROcp2_96*S7
    ROcp2_77 = ROcp2_16*S7+ROcp2_76*C7
    ROcp2_87 = ROcp2_26*S7+ROcp2_86*C7
    ROcp2_97 = ROcp2_36*S7+ROcp2_96*C7
    ROcp2_18 = ROcp2_17*C8-ROcp2_77*S8
    ROcp2_28 = ROcp2_27*C8-ROcp2_87*S8
    ROcp2_38 = ROcp2_37*C8-ROcp2_97*S8
    ROcp2_78 = ROcp2_17*S8+ROcp2_77*C8
    ROcp2_88 = ROcp2_27*S8+ROcp2_87*C8
    ROcp2_98 = ROcp2_37*S8+ROcp2_97*C8
    ROcp2_110 = ROcp2_18*C10+ROcp2_45*S10
    ROcp2_210 = ROcp2_28*C10+ROcp2_55*S10
    ROcp2_310 = ROcp2_38*C10+S10*S5
    ROcp2_410 = -ROcp2_18*S10+ROcp2_45*C10
    ROcp2_510 = -ROcp2_28*S10+ROcp2_55*C10
    ROcp2_610 = -ROcp2_38*S10+C10*S5
    ROcp2_111 = ROcp2_110*C11-ROcp2_78*S11
    ROcp2_211 = ROcp2_210*C11-ROcp2_88*S11
    ROcp2_311 = ROcp2_310*C11-ROcp2_98*S11
    ROcp2_711 = ROcp2_110*S11+ROcp2_78*C11
    ROcp2_811 = ROcp2_210*S11+ROcp2_88*C11
    ROcp2_911 = ROcp2_310*S11+ROcp2_98*C11
    ROcp2_112 = ROcp2_111*C12-ROcp2_711*S12
    ROcp2_212 = ROcp2_211*C12-ROcp2_811*S12
    ROcp2_312 = ROcp2_311*C12-ROcp2_911*S12
    ROcp2_712 = ROcp2_111*S12+ROcp2_711*C12
    ROcp2_812 = ROcp2_211*S12+ROcp2_811*C12
    ROcp2_912 = ROcp2_311*S12+ROcp2_911*C12
    ROcp2_113 = ROcp2_112*C13-ROcp2_712*S13
    ROcp2_213 = ROcp2_212*C13-ROcp2_812*S13
    ROcp2_313 = ROcp2_312*C13-ROcp2_912*S13
    ROcp2_713 = ROcp2_112*S13+ROcp2_712*C13
    ROcp2_813 = ROcp2_212*S13+ROcp2_812*C13
    ROcp2_913 = ROcp2_312*S13+ROcp2_912*C13
    ROcp2_114 = ROcp2_113*C14-ROcp2_713*S14
    ROcp2_214 = ROcp2_213*C14-ROcp2_813*S14
    ROcp2_314 = ROcp2_313*C14-ROcp2_913*S14
    ROcp2_714 = ROcp2_113*S14+ROcp2_713*C14
    ROcp2_814 = ROcp2_213*S14+ROcp2_813*C14
    ROcp2_914 = ROcp2_313*S14+ROcp2_913*C14
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
    RLcp2_17 = ROcp2_16*s.dpt[1,1]+ROcp2_76*s.dpt[3,1]
    RLcp2_27 = ROcp2_26*s.dpt[1,1]+ROcp2_86*s.dpt[3,1]
    RLcp2_37 = ROcp2_36*s.dpt[1,1]+ROcp2_96*s.dpt[3,1]
    POcp2_17 = q[1]+RLcp2_17
    POcp2_27 = q[2]+RLcp2_27
    POcp2_37 = q[3]+RLcp2_37
    OMcp2_17 = OMcp2_16+qd[7]*ROcp2_45
    OMcp2_27 = OMcp2_26+qd[7]*ROcp2_55
    OMcp2_37 = OMcp2_36+qd[7]*S5
    ORcp2_17 = OMcp2_26*RLcp2_37-OMcp2_36*RLcp2_27
    ORcp2_27 = -OMcp2_16*RLcp2_37+OMcp2_36*RLcp2_17
    ORcp2_37 = OMcp2_16*RLcp2_27-OMcp2_26*RLcp2_17
    VIcp2_17 = qd[1]+ORcp2_17
    VIcp2_27 = qd[2]+ORcp2_27
    VIcp2_37 = qd[3]+ORcp2_37
    OPcp2_17 = OPcp2_16+qd[7]*(OMcp2_26*S5-OMcp2_36*ROcp2_55)+qdd[7]*ROcp2_45
    OPcp2_27 = OPcp2_26+qd[7]*(-OMcp2_16*S5+OMcp2_36*ROcp2_45)+qdd[7]*ROcp2_55
    OPcp2_37 = OPcp2_36+qd[7]*(OMcp2_16*ROcp2_55-OMcp2_26*ROcp2_45)+qdd[7]*S5
    ACcp2_17 = qdd[1]+OMcp2_26*ORcp2_37-OMcp2_36*ORcp2_27+OPcp2_26*RLcp2_37-OPcp2_36*RLcp2_27
    ACcp2_27 = qdd[2]-OMcp2_16*ORcp2_37+OMcp2_36*ORcp2_17-OPcp2_16*RLcp2_37+OPcp2_36*RLcp2_17
    ACcp2_37 = qdd[3]+OMcp2_16*ORcp2_27-OMcp2_26*ORcp2_17+OPcp2_16*RLcp2_27-OPcp2_26*RLcp2_17
    RLcp2_18 = ROcp2_17*s.dpt[1,2]
    RLcp2_28 = ROcp2_27*s.dpt[1,2]
    RLcp2_38 = ROcp2_37*s.dpt[1,2]
    POcp2_18 = POcp2_17+RLcp2_18
    POcp2_28 = POcp2_27+RLcp2_28
    POcp2_38 = POcp2_37+RLcp2_38
    OMcp2_18 = OMcp2_17+qd[8]*ROcp2_45
    OMcp2_28 = OMcp2_27+qd[8]*ROcp2_55
    OMcp2_38 = OMcp2_37+qd[8]*S5
    ORcp2_18 = OMcp2_27*RLcp2_38-OMcp2_37*RLcp2_28
    ORcp2_28 = -OMcp2_17*RLcp2_38+OMcp2_37*RLcp2_18
    ORcp2_38 = OMcp2_17*RLcp2_28-OMcp2_27*RLcp2_18
    VIcp2_18 = ORcp2_18+VIcp2_17
    VIcp2_28 = ORcp2_28+VIcp2_27
    VIcp2_38 = ORcp2_38+VIcp2_37
    OPcp2_18 = OPcp2_17+qd[8]*(OMcp2_27*S5-OMcp2_37*ROcp2_55)+qdd[8]*ROcp2_45
    OPcp2_28 = OPcp2_27+qd[8]*(-OMcp2_17*S5+OMcp2_37*ROcp2_45)+qdd[8]*ROcp2_55
    OPcp2_38 = OPcp2_37+qd[8]*(OMcp2_17*ROcp2_55-OMcp2_27*ROcp2_45)+qdd[8]*S5
    ACcp2_18 = ACcp2_17+OMcp2_27*ORcp2_38-OMcp2_37*ORcp2_28+OPcp2_27*RLcp2_38-OPcp2_37*RLcp2_28
    ACcp2_28 = ACcp2_27-OMcp2_17*ORcp2_38+OMcp2_37*ORcp2_18-OPcp2_17*RLcp2_38+OPcp2_37*RLcp2_18
    ACcp2_38 = ACcp2_37+OMcp2_17*ORcp2_28-OMcp2_27*ORcp2_18+OPcp2_17*RLcp2_28-OPcp2_27*RLcp2_18
    RLcp2_19 = ROcp2_78*s.dpt[3,4]
    RLcp2_29 = ROcp2_88*s.dpt[3,4]
    RLcp2_39 = ROcp2_98*s.dpt[3,4]
    POcp2_19 = POcp2_18+RLcp2_19
    POcp2_29 = POcp2_28+RLcp2_29
    POcp2_39 = POcp2_38+RLcp2_39
    OMcp2_19 = OMcp2_18+qd[10]*ROcp2_78
    OMcp2_29 = OMcp2_28+qd[10]*ROcp2_88
    OMcp2_39 = OMcp2_38+qd[10]*ROcp2_98
    ORcp2_19 = OMcp2_28*RLcp2_39-OMcp2_38*RLcp2_29
    ORcp2_29 = -OMcp2_18*RLcp2_39+OMcp2_38*RLcp2_19
    ORcp2_39 = OMcp2_18*RLcp2_29-OMcp2_28*RLcp2_19
    VIcp2_19 = ORcp2_19+VIcp2_18
    VIcp2_29 = ORcp2_29+VIcp2_28
    VIcp2_39 = ORcp2_39+VIcp2_38
    OPcp2_19 = OPcp2_18+qd[10]*(OMcp2_28*ROcp2_98-OMcp2_38*ROcp2_88)+qdd[10]*ROcp2_78
    OPcp2_29 = OPcp2_28+qd[10]*(-OMcp2_18*ROcp2_98+OMcp2_38*ROcp2_78)+qdd[10]*ROcp2_88
    OPcp2_39 = OPcp2_38+qd[10]*(OMcp2_18*ROcp2_88-OMcp2_28*ROcp2_78)+qdd[10]*ROcp2_98
    ACcp2_19 = ACcp2_18+OMcp2_28*ORcp2_39-OMcp2_38*ORcp2_29+OPcp2_28*RLcp2_39-OPcp2_38*RLcp2_29
    ACcp2_29 = ACcp2_28-OMcp2_18*ORcp2_39+OMcp2_38*ORcp2_19-OPcp2_18*RLcp2_39+OPcp2_38*RLcp2_19
    ACcp2_39 = ACcp2_38+OMcp2_18*ORcp2_29-OMcp2_28*ORcp2_19+OPcp2_18*RLcp2_29-OPcp2_28*RLcp2_19
    OMcp2_110 = OMcp2_19+qd[11]*ROcp2_410
    OMcp2_210 = OMcp2_29+qd[11]*ROcp2_510
    OMcp2_310 = OMcp2_39+qd[11]*ROcp2_610
    OPcp2_110 = OPcp2_19+qd[11]*(OMcp2_29*ROcp2_610-OMcp2_39*ROcp2_510)+qdd[11]*ROcp2_410
    OPcp2_210 = OPcp2_29+qd[11]*(-OMcp2_19*ROcp2_610+OMcp2_39*ROcp2_410)+qdd[11]*ROcp2_510
    OPcp2_310 = OPcp2_39+qd[11]*(OMcp2_19*ROcp2_510-OMcp2_29*ROcp2_410)+qdd[11]*ROcp2_610
    RLcp2_111 = ROcp2_711*s.dpt[3,6]
    RLcp2_211 = ROcp2_811*s.dpt[3,6]
    RLcp2_311 = ROcp2_911*s.dpt[3,6]
    POcp2_111 = POcp2_19+RLcp2_111
    POcp2_211 = POcp2_29+RLcp2_211
    POcp2_311 = POcp2_39+RLcp2_311
    OMcp2_111 = OMcp2_110+qd[12]*ROcp2_410
    OMcp2_211 = OMcp2_210+qd[12]*ROcp2_510
    OMcp2_311 = OMcp2_310+qd[12]*ROcp2_610
    ORcp2_111 = OMcp2_210*RLcp2_311-OMcp2_310*RLcp2_211
    ORcp2_211 = -OMcp2_110*RLcp2_311+OMcp2_310*RLcp2_111
    ORcp2_311 = OMcp2_110*RLcp2_211-OMcp2_210*RLcp2_111
    VIcp2_111 = ORcp2_111+VIcp2_19
    VIcp2_211 = ORcp2_211+VIcp2_29
    VIcp2_311 = ORcp2_311+VIcp2_39
    OPcp2_111 = OPcp2_110+qd[12]*(OMcp2_210*ROcp2_610-OMcp2_310*ROcp2_510)+qdd[12]*ROcp2_410
    OPcp2_211 = OPcp2_210+qd[12]*(-OMcp2_110*ROcp2_610+OMcp2_310*ROcp2_410)+qdd[12]*ROcp2_510
    OPcp2_311 = OPcp2_310+qd[12]*(OMcp2_110*ROcp2_510-OMcp2_210*ROcp2_410)+qdd[12]*ROcp2_610
    ACcp2_111 = ACcp2_19+OMcp2_210*ORcp2_311-OMcp2_310*ORcp2_211+OPcp2_210*RLcp2_311-OPcp2_310*RLcp2_211
    ACcp2_211 = ACcp2_29-OMcp2_110*ORcp2_311+OMcp2_310*ORcp2_111-OPcp2_110*RLcp2_311+OPcp2_310*RLcp2_111
    ACcp2_311 = ACcp2_39+OMcp2_110*ORcp2_211-OMcp2_210*ORcp2_111+OPcp2_110*RLcp2_211-OPcp2_210*RLcp2_111
    RLcp2_112 = ROcp2_112*s.dpt[1,7]+ROcp2_410*s.dpt[2,7]+ROcp2_712*s.dpt[3,7]
    RLcp2_212 = ROcp2_212*s.dpt[1,7]+ROcp2_510*s.dpt[2,7]+ROcp2_812*s.dpt[3,7]
    RLcp2_312 = ROcp2_312*s.dpt[1,7]+ROcp2_610*s.dpt[2,7]+ROcp2_912*s.dpt[3,7]
    POcp2_112 = POcp2_111+RLcp2_112
    POcp2_212 = POcp2_211+RLcp2_212
    POcp2_312 = POcp2_311+RLcp2_312
    OMcp2_112 = OMcp2_111+qd[13]*ROcp2_410
    OMcp2_212 = OMcp2_211+qd[13]*ROcp2_510
    OMcp2_312 = OMcp2_311+qd[13]*ROcp2_610
    ORcp2_112 = OMcp2_211*RLcp2_312-OMcp2_311*RLcp2_212
    ORcp2_212 = -OMcp2_111*RLcp2_312+OMcp2_311*RLcp2_112
    ORcp2_312 = OMcp2_111*RLcp2_212-OMcp2_211*RLcp2_112
    VIcp2_112 = ORcp2_112+VIcp2_111
    VIcp2_212 = ORcp2_212+VIcp2_211
    VIcp2_312 = ORcp2_312+VIcp2_311
    OPcp2_112 = OPcp2_111+qd[13]*(OMcp2_211*ROcp2_610-OMcp2_311*ROcp2_510)+qdd[13]*ROcp2_410
    OPcp2_212 = OPcp2_211+qd[13]*(-OMcp2_111*ROcp2_610+OMcp2_311*ROcp2_410)+qdd[13]*ROcp2_510
    OPcp2_312 = OPcp2_311+qd[13]*(OMcp2_111*ROcp2_510-OMcp2_211*ROcp2_410)+qdd[13]*ROcp2_610
    ACcp2_112 = ACcp2_111+OMcp2_211*ORcp2_312-OMcp2_311*ORcp2_212+OPcp2_211*RLcp2_312-OPcp2_311*RLcp2_212
    ACcp2_212 = ACcp2_211-OMcp2_111*ORcp2_312+OMcp2_311*ORcp2_112-OPcp2_111*RLcp2_312+OPcp2_311*RLcp2_112
    ACcp2_312 = ACcp2_311+OMcp2_111*ORcp2_212-OMcp2_211*ORcp2_112+OPcp2_111*RLcp2_212-OPcp2_211*RLcp2_112
    RLcp2_113 = ROcp2_113*s.dpt[1,12]
    RLcp2_213 = ROcp2_213*s.dpt[1,12]
    RLcp2_313 = ROcp2_313*s.dpt[1,12]
    POcp2_113 = POcp2_112+RLcp2_113
    POcp2_213 = POcp2_212+RLcp2_213
    POcp2_313 = POcp2_312+RLcp2_313
    OMcp2_113 = OMcp2_112+qd[14]*ROcp2_410
    OMcp2_213 = OMcp2_212+qd[14]*ROcp2_510
    OMcp2_313 = OMcp2_312+qd[14]*ROcp2_610
    ORcp2_113 = OMcp2_212*RLcp2_313-OMcp2_312*RLcp2_213
    ORcp2_213 = -OMcp2_112*RLcp2_313+OMcp2_312*RLcp2_113
    ORcp2_313 = OMcp2_112*RLcp2_213-OMcp2_212*RLcp2_113
    VIcp2_113 = ORcp2_113+VIcp2_112
    VIcp2_213 = ORcp2_213+VIcp2_212
    VIcp2_313 = ORcp2_313+VIcp2_312
    OPcp2_113 = OPcp2_112+qd[14]*(OMcp2_212*ROcp2_610-OMcp2_312*ROcp2_510)+qdd[14]*ROcp2_410
    OPcp2_213 = OPcp2_212+qd[14]*(-OMcp2_112*ROcp2_610+OMcp2_312*ROcp2_410)+qdd[14]*ROcp2_510
    OPcp2_313 = OPcp2_312+qd[14]*(OMcp2_112*ROcp2_510-OMcp2_212*ROcp2_410)+qdd[14]*ROcp2_610
    ACcp2_113 = ACcp2_112+OMcp2_212*ORcp2_313-OMcp2_312*ORcp2_213+OPcp2_212*RLcp2_313-OPcp2_312*RLcp2_213
    ACcp2_213 = ACcp2_212-OMcp2_112*ORcp2_313+OMcp2_312*ORcp2_113-OPcp2_112*RLcp2_313+OPcp2_312*RLcp2_113
    ACcp2_313 = ACcp2_312+OMcp2_112*ORcp2_213-OMcp2_212*ORcp2_113+OPcp2_112*RLcp2_213-OPcp2_212*RLcp2_113
    PxF2[1] = POcp2_113
    PxF2[2] = POcp2_213
    PxF2[3] = POcp2_313
    RxF2[1,1] = ROcp2_114
    RxF2[1,2] = ROcp2_214
    RxF2[1,3] = ROcp2_314
    RxF2[2,1] = ROcp2_410
    RxF2[2,2] = ROcp2_510
    RxF2[2,3] = ROcp2_610
    RxF2[3,1] = ROcp2_714
    RxF2[3,2] = ROcp2_814
    RxF2[3,3] = ROcp2_914
    VxF2[1] = VIcp2_113
    VxF2[2] = VIcp2_213
    VxF2[3] = VIcp2_313
    OMxF2[1] = OMcp2_113
    OMxF2[2] = OMcp2_213
    OMxF2[3] = OMcp2_313
    AxF2[1] = ACcp2_113
    AxF2[2] = ACcp2_213
    AxF2[3] = ACcp2_313
    OMPxF2[1] = OPcp2_113
    OMPxF2[2] = OPcp2_213
    OMPxF2[3] = OPcp2_313
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
    ROcp3_18 = ROcp3_17*C8-ROcp3_77*S8
    ROcp3_28 = ROcp3_27*C8-ROcp3_87*S8
    ROcp3_38 = ROcp3_37*C8-ROcp3_97*S8
    ROcp3_78 = ROcp3_17*S8+ROcp3_77*C8
    ROcp3_88 = ROcp3_27*S8+ROcp3_87*C8
    ROcp3_98 = ROcp3_37*S8+ROcp3_97*C8
    ROcp3_110 = ROcp3_18*C10+ROcp3_45*S10
    ROcp3_210 = ROcp3_28*C10+ROcp3_55*S10
    ROcp3_310 = ROcp3_38*C10+S10*S5
    ROcp3_410 = -ROcp3_18*S10+ROcp3_45*C10
    ROcp3_510 = -ROcp3_28*S10+ROcp3_55*C10
    ROcp3_610 = -ROcp3_38*S10+C10*S5
    ROcp3_111 = ROcp3_110*C11-ROcp3_78*S11
    ROcp3_211 = ROcp3_210*C11-ROcp3_88*S11
    ROcp3_311 = ROcp3_310*C11-ROcp3_98*S11
    ROcp3_711 = ROcp3_110*S11+ROcp3_78*C11
    ROcp3_811 = ROcp3_210*S11+ROcp3_88*C11
    ROcp3_911 = ROcp3_310*S11+ROcp3_98*C11
    ROcp3_112 = ROcp3_111*C12-ROcp3_711*S12
    ROcp3_212 = ROcp3_211*C12-ROcp3_811*S12
    ROcp3_312 = ROcp3_311*C12-ROcp3_911*S12
    ROcp3_712 = ROcp3_111*S12+ROcp3_711*C12
    ROcp3_812 = ROcp3_211*S12+ROcp3_811*C12
    ROcp3_912 = ROcp3_311*S12+ROcp3_911*C12
    ROcp3_115 = ROcp3_112*C15-ROcp3_712*S15
    ROcp3_215 = ROcp3_212*C15-ROcp3_812*S15
    ROcp3_315 = ROcp3_312*C15-ROcp3_912*S15
    ROcp3_715 = ROcp3_112*S15+ROcp3_712*C15
    ROcp3_815 = ROcp3_212*S15+ROcp3_812*C15
    ROcp3_915 = ROcp3_312*S15+ROcp3_912*C15
    ROcp3_116 = ROcp3_115*C16-ROcp3_715*S16
    ROcp3_216 = ROcp3_215*C16-ROcp3_815*S16
    ROcp3_316 = ROcp3_315*C16-ROcp3_915*S16
    ROcp3_716 = ROcp3_115*S16+ROcp3_715*C16
    ROcp3_816 = ROcp3_215*S16+ROcp3_815*C16
    ROcp3_916 = ROcp3_315*S16+ROcp3_915*C16
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
    RLcp3_17 = ROcp3_16*s.dpt[1,1]+ROcp3_76*s.dpt[3,1]
    RLcp3_27 = ROcp3_26*s.dpt[1,1]+ROcp3_86*s.dpt[3,1]
    RLcp3_37 = ROcp3_36*s.dpt[1,1]+ROcp3_96*s.dpt[3,1]
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
    RLcp3_18 = ROcp3_17*s.dpt[1,2]
    RLcp3_28 = ROcp3_27*s.dpt[1,2]
    RLcp3_38 = ROcp3_37*s.dpt[1,2]
    POcp3_18 = POcp3_17+RLcp3_18
    POcp3_28 = POcp3_27+RLcp3_28
    POcp3_38 = POcp3_37+RLcp3_38
    OMcp3_18 = OMcp3_17+qd[8]*ROcp3_45
    OMcp3_28 = OMcp3_27+qd[8]*ROcp3_55
    OMcp3_38 = OMcp3_37+qd[8]*S5
    ORcp3_18 = OMcp3_27*RLcp3_38-OMcp3_37*RLcp3_28
    ORcp3_28 = -OMcp3_17*RLcp3_38+OMcp3_37*RLcp3_18
    ORcp3_38 = OMcp3_17*RLcp3_28-OMcp3_27*RLcp3_18
    VIcp3_18 = ORcp3_18+VIcp3_17
    VIcp3_28 = ORcp3_28+VIcp3_27
    VIcp3_38 = ORcp3_38+VIcp3_37
    OPcp3_18 = OPcp3_17+qd[8]*(OMcp3_27*S5-OMcp3_37*ROcp3_55)+qdd[8]*ROcp3_45
    OPcp3_28 = OPcp3_27+qd[8]*(-OMcp3_17*S5+OMcp3_37*ROcp3_45)+qdd[8]*ROcp3_55
    OPcp3_38 = OPcp3_37+qd[8]*(OMcp3_17*ROcp3_55-OMcp3_27*ROcp3_45)+qdd[8]*S5
    ACcp3_18 = ACcp3_17+OMcp3_27*ORcp3_38-OMcp3_37*ORcp3_28+OPcp3_27*RLcp3_38-OPcp3_37*RLcp3_28
    ACcp3_28 = ACcp3_27-OMcp3_17*ORcp3_38+OMcp3_37*ORcp3_18-OPcp3_17*RLcp3_38+OPcp3_37*RLcp3_18
    ACcp3_38 = ACcp3_37+OMcp3_17*ORcp3_28-OMcp3_27*ORcp3_18+OPcp3_17*RLcp3_28-OPcp3_27*RLcp3_18
    RLcp3_19 = ROcp3_78*s.dpt[3,4]
    RLcp3_29 = ROcp3_88*s.dpt[3,4]
    RLcp3_39 = ROcp3_98*s.dpt[3,4]
    POcp3_19 = POcp3_18+RLcp3_19
    POcp3_29 = POcp3_28+RLcp3_29
    POcp3_39 = POcp3_38+RLcp3_39
    OMcp3_19 = OMcp3_18+qd[10]*ROcp3_78
    OMcp3_29 = OMcp3_28+qd[10]*ROcp3_88
    OMcp3_39 = OMcp3_38+qd[10]*ROcp3_98
    ORcp3_19 = OMcp3_28*RLcp3_39-OMcp3_38*RLcp3_29
    ORcp3_29 = -OMcp3_18*RLcp3_39+OMcp3_38*RLcp3_19
    ORcp3_39 = OMcp3_18*RLcp3_29-OMcp3_28*RLcp3_19
    VIcp3_19 = ORcp3_19+VIcp3_18
    VIcp3_29 = ORcp3_29+VIcp3_28
    VIcp3_39 = ORcp3_39+VIcp3_38
    OPcp3_19 = OPcp3_18+qd[10]*(OMcp3_28*ROcp3_98-OMcp3_38*ROcp3_88)+qdd[10]*ROcp3_78
    OPcp3_29 = OPcp3_28+qd[10]*(-OMcp3_18*ROcp3_98+OMcp3_38*ROcp3_78)+qdd[10]*ROcp3_88
    OPcp3_39 = OPcp3_38+qd[10]*(OMcp3_18*ROcp3_88-OMcp3_28*ROcp3_78)+qdd[10]*ROcp3_98
    ACcp3_19 = ACcp3_18+OMcp3_28*ORcp3_39-OMcp3_38*ORcp3_29+OPcp3_28*RLcp3_39-OPcp3_38*RLcp3_29
    ACcp3_29 = ACcp3_28-OMcp3_18*ORcp3_39+OMcp3_38*ORcp3_19-OPcp3_18*RLcp3_39+OPcp3_38*RLcp3_19
    ACcp3_39 = ACcp3_38+OMcp3_18*ORcp3_29-OMcp3_28*ORcp3_19+OPcp3_18*RLcp3_29-OPcp3_28*RLcp3_19
    OMcp3_110 = OMcp3_19+qd[11]*ROcp3_410
    OMcp3_210 = OMcp3_29+qd[11]*ROcp3_510
    OMcp3_310 = OMcp3_39+qd[11]*ROcp3_610
    OPcp3_110 = OPcp3_19+qd[11]*(OMcp3_29*ROcp3_610-OMcp3_39*ROcp3_510)+qdd[11]*ROcp3_410
    OPcp3_210 = OPcp3_29+qd[11]*(-OMcp3_19*ROcp3_610+OMcp3_39*ROcp3_410)+qdd[11]*ROcp3_510
    OPcp3_310 = OPcp3_39+qd[11]*(OMcp3_19*ROcp3_510-OMcp3_29*ROcp3_410)+qdd[11]*ROcp3_610
    RLcp3_111 = ROcp3_711*s.dpt[3,6]
    RLcp3_211 = ROcp3_811*s.dpt[3,6]
    RLcp3_311 = ROcp3_911*s.dpt[3,6]
    POcp3_111 = POcp3_19+RLcp3_111
    POcp3_211 = POcp3_29+RLcp3_211
    POcp3_311 = POcp3_39+RLcp3_311
    OMcp3_111 = OMcp3_110+qd[12]*ROcp3_410
    OMcp3_211 = OMcp3_210+qd[12]*ROcp3_510
    OMcp3_311 = OMcp3_310+qd[12]*ROcp3_610
    ORcp3_111 = OMcp3_210*RLcp3_311-OMcp3_310*RLcp3_211
    ORcp3_211 = -OMcp3_110*RLcp3_311+OMcp3_310*RLcp3_111
    ORcp3_311 = OMcp3_110*RLcp3_211-OMcp3_210*RLcp3_111
    VIcp3_111 = ORcp3_111+VIcp3_19
    VIcp3_211 = ORcp3_211+VIcp3_29
    VIcp3_311 = ORcp3_311+VIcp3_39
    OPcp3_111 = OPcp3_110+qd[12]*(OMcp3_210*ROcp3_610-OMcp3_310*ROcp3_510)+qdd[12]*ROcp3_410
    OPcp3_211 = OPcp3_210+qd[12]*(-OMcp3_110*ROcp3_610+OMcp3_310*ROcp3_410)+qdd[12]*ROcp3_510
    OPcp3_311 = OPcp3_310+qd[12]*(OMcp3_110*ROcp3_510-OMcp3_210*ROcp3_410)+qdd[12]*ROcp3_610
    ACcp3_111 = ACcp3_19+OMcp3_210*ORcp3_311-OMcp3_310*ORcp3_211+OPcp3_210*RLcp3_311-OPcp3_310*RLcp3_211
    ACcp3_211 = ACcp3_29-OMcp3_110*ORcp3_311+OMcp3_310*ORcp3_111-OPcp3_110*RLcp3_311+OPcp3_310*RLcp3_111
    ACcp3_311 = ACcp3_39+OMcp3_110*ORcp3_211-OMcp3_210*ORcp3_111+OPcp3_110*RLcp3_211-OPcp3_210*RLcp3_111
    RLcp3_112 = ROcp3_112*s.dpt[1,8]+ROcp3_410*s.dpt[2,8]+ROcp3_712*s.dpt[3,8]
    RLcp3_212 = ROcp3_212*s.dpt[1,8]+ROcp3_510*s.dpt[2,8]+ROcp3_812*s.dpt[3,8]
    RLcp3_312 = ROcp3_312*s.dpt[1,8]+ROcp3_610*s.dpt[2,8]+ROcp3_912*s.dpt[3,8]
    POcp3_112 = POcp3_111+RLcp3_112
    POcp3_212 = POcp3_211+RLcp3_212
    POcp3_312 = POcp3_311+RLcp3_312
    OMcp3_112 = OMcp3_111+qd[15]*ROcp3_410
    OMcp3_212 = OMcp3_211+qd[15]*ROcp3_510
    OMcp3_312 = OMcp3_311+qd[15]*ROcp3_610
    ORcp3_112 = OMcp3_211*RLcp3_312-OMcp3_311*RLcp3_212
    ORcp3_212 = -OMcp3_111*RLcp3_312+OMcp3_311*RLcp3_112
    ORcp3_312 = OMcp3_111*RLcp3_212-OMcp3_211*RLcp3_112
    VIcp3_112 = ORcp3_112+VIcp3_111
    VIcp3_212 = ORcp3_212+VIcp3_211
    VIcp3_312 = ORcp3_312+VIcp3_311
    OPcp3_112 = OPcp3_111+qd[15]*(OMcp3_211*ROcp3_610-OMcp3_311*ROcp3_510)+qdd[15]*ROcp3_410
    OPcp3_212 = OPcp3_211+qd[15]*(-OMcp3_111*ROcp3_610+OMcp3_311*ROcp3_410)+qdd[15]*ROcp3_510
    OPcp3_312 = OPcp3_311+qd[15]*(OMcp3_111*ROcp3_510-OMcp3_211*ROcp3_410)+qdd[15]*ROcp3_610
    ACcp3_112 = ACcp3_111+OMcp3_211*ORcp3_312-OMcp3_311*ORcp3_212+OPcp3_211*RLcp3_312-OPcp3_311*RLcp3_212
    ACcp3_212 = ACcp3_211-OMcp3_111*ORcp3_312+OMcp3_311*ORcp3_112-OPcp3_111*RLcp3_312+OPcp3_311*RLcp3_112
    ACcp3_312 = ACcp3_311+OMcp3_111*ORcp3_212-OMcp3_211*ORcp3_112+OPcp3_111*RLcp3_212-OPcp3_211*RLcp3_112
    RLcp3_113 = ROcp3_115*s.dpt[1,15]
    RLcp3_213 = ROcp3_215*s.dpt[1,15]
    RLcp3_313 = ROcp3_315*s.dpt[1,15]
    POcp3_113 = POcp3_112+RLcp3_113
    POcp3_213 = POcp3_212+RLcp3_213
    POcp3_313 = POcp3_312+RLcp3_313
    OMcp3_113 = OMcp3_112+qd[16]*ROcp3_410
    OMcp3_213 = OMcp3_212+qd[16]*ROcp3_510
    OMcp3_313 = OMcp3_312+qd[16]*ROcp3_610
    ORcp3_113 = OMcp3_212*RLcp3_313-OMcp3_312*RLcp3_213
    ORcp3_213 = -OMcp3_112*RLcp3_313+OMcp3_312*RLcp3_113
    ORcp3_313 = OMcp3_112*RLcp3_213-OMcp3_212*RLcp3_113
    VIcp3_113 = ORcp3_113+VIcp3_112
    VIcp3_213 = ORcp3_213+VIcp3_212
    VIcp3_313 = ORcp3_313+VIcp3_312
    OPcp3_113 = OPcp3_112+qd[16]*(OMcp3_212*ROcp3_610-OMcp3_312*ROcp3_510)+qdd[16]*ROcp3_410
    OPcp3_213 = OPcp3_212+qd[16]*(-OMcp3_112*ROcp3_610+OMcp3_312*ROcp3_410)+qdd[16]*ROcp3_510
    OPcp3_313 = OPcp3_312+qd[16]*(OMcp3_112*ROcp3_510-OMcp3_212*ROcp3_410)+qdd[16]*ROcp3_610
    ACcp3_113 = ACcp3_112+OMcp3_212*ORcp3_313-OMcp3_312*ORcp3_213+OPcp3_212*RLcp3_313-OPcp3_312*RLcp3_213
    ACcp3_213 = ACcp3_212-OMcp3_112*ORcp3_313+OMcp3_312*ORcp3_113-OPcp3_112*RLcp3_313+OPcp3_312*RLcp3_113
    ACcp3_313 = ACcp3_312+OMcp3_112*ORcp3_213-OMcp3_212*ORcp3_113+OPcp3_112*RLcp3_213-OPcp3_212*RLcp3_113
    PxF3[1] = POcp3_113
    PxF3[2] = POcp3_213
    PxF3[3] = POcp3_313
    RxF3[1,1] = ROcp3_116
    RxF3[1,2] = ROcp3_216
    RxF3[1,3] = ROcp3_316
    RxF3[2,1] = ROcp3_410
    RxF3[2,2] = ROcp3_510
    RxF3[2,3] = ROcp3_610
    RxF3[3,1] = ROcp3_716
    RxF3[3,2] = ROcp3_816
    RxF3[3,3] = ROcp3_916
    VxF3[1] = VIcp3_113
    VxF3[2] = VIcp3_213
    VxF3[3] = VIcp3_313
    OMxF3[1] = OMcp3_113
    OMxF3[2] = OMcp3_213
    OMxF3[3] = OMcp3_313
    AxF3[1] = ACcp3_113
    AxF3[2] = ACcp3_213
    AxF3[3] = ACcp3_313
    OMPxF3[1] = OPcp3_113
    OMPxF3[2] = OPcp3_213
    OMPxF3[3] = OPcp3_313
 
# Sensor Forces 

    SWr1 = s.user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1)
    SWr2 = s.user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2)
    SWr3 = s.user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3)
    xfrc11 = RxF1[1,1]*SWr1[1]+RxF1[1,2]*SWr1[2]+RxF1[1,3]*SWr1[3]
    xfrc21 = RxF1[2,1]*SWr1[1]+RxF1[2,2]*SWr1[2]+RxF1[2,3]*SWr1[3]
    xfrc31 = RxF1[3,1]*SWr1[1]+RxF1[3,2]*SWr1[2]+RxF1[3,3]*SWr1[3]
    xtrq11 = RxF1[1,1]*SWr1[4]+RxF1[1,2]*SWr1[5]+RxF1[1,3]*SWr1[6]
    xtrq21 = RxF1[2,1]*SWr1[4]+RxF1[2,2]*SWr1[5]+RxF1[2,3]*SWr1[6]
    xtrq31 = RxF1[3,1]*SWr1[4]+RxF1[3,2]*SWr1[5]+RxF1[3,3]*SWr1[6]
    trqext_1_9_0 = xtrq11-xfrc21*SWr1[9]+xfrc31*SWr1[8]
    trqext_2_9_0 = xtrq21+xfrc11*SWr1[9]-xfrc31*SWr1[7]
    trqext_3_9_0 = xtrq31-xfrc11*SWr1[8]+xfrc21*SWr1[7]
    xfrc12 = RxF2[1,1]*SWr2[1]+RxF2[1,2]*SWr2[2]+RxF2[1,3]*SWr2[3]
    xfrc22 = RxF2[2,1]*SWr2[1]+RxF2[2,2]*SWr2[2]+RxF2[2,3]*SWr2[3]
    xfrc32 = RxF2[3,1]*SWr2[1]+RxF2[3,2]*SWr2[2]+RxF2[3,3]*SWr2[3]
    xtrq12 = RxF2[1,1]*SWr2[4]+RxF2[1,2]*SWr2[5]+RxF2[1,3]*SWr2[6]
    xtrq22 = RxF2[2,1]*SWr2[4]+RxF2[2,2]*SWr2[5]+RxF2[2,3]*SWr2[6]
    xtrq32 = RxF2[3,1]*SWr2[4]+RxF2[3,2]*SWr2[5]+RxF2[3,3]*SWr2[6]
    trqext_1_14_1 = xtrq12-xfrc22*SWr2[9]+xfrc32*SWr2[8]
    trqext_2_14_1 = xtrq22+xfrc12*SWr2[9]-xfrc32*SWr2[7]
    trqext_3_14_1 = xtrq32-xfrc12*SWr2[8]+xfrc22*SWr2[7]
    xfrc13 = RxF3[1,1]*SWr3[1]+RxF3[1,2]*SWr3[2]+RxF3[1,3]*SWr3[3]
    xfrc23 = RxF3[2,1]*SWr3[1]+RxF3[2,2]*SWr3[2]+RxF3[2,3]*SWr3[3]
    xfrc33 = RxF3[3,1]*SWr3[1]+RxF3[3,2]*SWr3[2]+RxF3[3,3]*SWr3[3]
    xtrq13 = RxF3[1,1]*SWr3[4]+RxF3[1,2]*SWr3[5]+RxF3[1,3]*SWr3[6]
    xtrq23 = RxF3[2,1]*SWr3[4]+RxF3[2,2]*SWr3[5]+RxF3[2,3]*SWr3[6]
    xtrq33 = RxF3[3,1]*SWr3[4]+RxF3[3,2]*SWr3[5]+RxF3[3,3]*SWr3[6]
    trqext_1_16_2 = xtrq13-xfrc23*SWr3[9]+xfrc33*SWr3[8]
    trqext_2_16_2 = xtrq23+xfrc13*SWr3[9]-xfrc33*SWr3[7]
    trqext_3_16_2 = xtrq33-xfrc13*SWr3[8]+xfrc23*SWr3[7]
 
# Symbolic model output

    frc[1,9] = s.frc[1,9]+xfrc11
    frc[2,9] = s.frc[2,9]+xfrc21
    frc[3,9] = s.frc[3,9]+xfrc31
    trq[1,9] = s.trq[1,9]+trqext_1_9_0
    trq[2,9] = s.trq[2,9]+trqext_2_9_0
    trq[3,9] = s.trq[3,9]+trqext_3_9_0
    frc[1,14] = s.frc[1,14]+xfrc12
    frc[2,14] = s.frc[2,14]+xfrc22
    frc[3,14] = s.frc[3,14]+xfrc32
    trq[1,14] = s.trq[1,14]+trqext_1_14_1
    trq[2,14] = s.trq[2,14]+trqext_2_14_1
    trq[3,14] = s.trq[3,14]+trqext_3_14_1
    frc[1,16] = s.frc[1,16]+xfrc13
    frc[2,16] = s.frc[2,16]+xfrc23
    frc[3,16] = s.frc[3,16]+xfrc33
    trq[1,16] = s.trq[1,16]+trqext_1_16_2
    trq[2,16] = s.trq[2,16]+trqext_2_16_2
    trq[3,16] = s.trq[3,16]+trqext_3_16_2

# Number of continuation lines = 0


