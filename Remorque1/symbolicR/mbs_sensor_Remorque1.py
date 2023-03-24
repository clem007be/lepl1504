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
#	==> Function: F6 - Sensors Kinematics
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos, sqrt

def sensor(sens, s, isens):
  q = s.q
  qd = s.qd
  qdd = s.qdd

  dpt = s.dpt
 
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
  S16 = sin(q[16])
  C16 = cos(q[16])
 
# Augmented Joint Position Vectors

  Dz153 = q[15]+s.dpt[3,4]
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

    ROcp1_15 = C4*C5
    ROcp1_25 = S4*C5
    ROcp1_75 = C4*S5
    ROcp1_85 = S4*S5
    ROcp1_16 = ROcp1_15*C6-ROcp1_75*S6
    ROcp1_26 = ROcp1_25*C6-ROcp1_85*S6
    ROcp1_36 = -C5*S6-S5*C6
    ROcp1_76 = ROcp1_15*S6+ROcp1_75*C6
    ROcp1_86 = ROcp1_25*S6+ROcp1_85*C6
    ROcp1_96 = C5*C6-S5*S6
    ROcp1_17 = ROcp1_16*C7-S4*S7
    ROcp1_27 = ROcp1_26*C7+C4*S7
    ROcp1_37 = ROcp1_36*C7
    ROcp1_47 = -ROcp1_16*S7-S4*C7
    ROcp1_57 = -ROcp1_26*S7+C4*C7
    ROcp1_67 = -ROcp1_36*S7
    ROcp1_18 = ROcp1_17*C8-ROcp1_76*S8
    ROcp1_28 = ROcp1_27*C8-ROcp1_86*S8
    ROcp1_38 = ROcp1_37*C8-ROcp1_96*S8
    ROcp1_78 = ROcp1_17*S8+ROcp1_76*C8
    ROcp1_88 = ROcp1_27*S8+ROcp1_86*C8
    ROcp1_98 = ROcp1_37*S8+ROcp1_96*C8
    ROcp1_19 = ROcp1_18*C9-ROcp1_78*S9
    ROcp1_29 = ROcp1_28*C9-ROcp1_88*S9
    ROcp1_39 = ROcp1_38*C9-ROcp1_98*S9
    ROcp1_79 = ROcp1_18*S9+ROcp1_78*C9
    ROcp1_89 = ROcp1_28*S9+ROcp1_88*C9
    ROcp1_99 = ROcp1_38*S9+ROcp1_98*C9
    ROcp1_110 = ROcp1_19*C10-ROcp1_79*S10
    ROcp1_210 = ROcp1_29*C10-ROcp1_89*S10
    ROcp1_310 = ROcp1_39*C10-ROcp1_99*S10
    ROcp1_710 = ROcp1_19*S10+ROcp1_79*C10
    ROcp1_810 = ROcp1_29*S10+ROcp1_89*C10
    ROcp1_910 = ROcp1_39*S10+ROcp1_99*C10
    ROcp1_111 = ROcp1_110*C11-ROcp1_710*S11
    ROcp1_211 = ROcp1_210*C11-ROcp1_810*S11
    ROcp1_311 = ROcp1_310*C11-ROcp1_910*S11
    ROcp1_711 = ROcp1_110*S11+ROcp1_710*C11
    ROcp1_811 = ROcp1_210*S11+ROcp1_810*C11
    ROcp1_911 = ROcp1_310*S11+ROcp1_910*C11
    RLcp1_15 = s.dpt[1,1]*C4
    RLcp1_25 = s.dpt[1,1]*S4
    POcp1_15 = RLcp1_15+q[1]
    POcp1_25 = RLcp1_25+q[2]
    POcp1_35 = q[3]+s.dpt[3,1]
    OMcp1_15 = -qd[5]*S4
    OMcp1_25 = qd[5]*C4
    ORcp1_15 = -RLcp1_25*qd[4]
    ORcp1_25 = RLcp1_15*qd[4]
    VIcp1_15 = ORcp1_15+qd[1]
    VIcp1_25 = ORcp1_25+qd[2]
    OPcp1_15 = -qdd[5]*S4-qd[4]*qd[5]*C4
    OPcp1_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    ACcp1_15 = qdd[1]-ORcp1_25*qd[4]-RLcp1_25*qdd[4]
    ACcp1_25 = qdd[2]+ORcp1_15*qd[4]+RLcp1_15*qdd[4]
    RLcp1_16 = ROcp1_15*s.dpt[1,2]
    RLcp1_26 = ROcp1_25*s.dpt[1,2]
    RLcp1_36 = -s.dpt[1,2]*S5
    POcp1_16 = POcp1_15+RLcp1_16
    POcp1_26 = POcp1_25+RLcp1_26
    POcp1_36 = POcp1_35+RLcp1_36
    OMcp1_16 = OMcp1_15-qd[6]*S4
    OMcp1_26 = OMcp1_25+qd[6]*C4
    ORcp1_16 = OMcp1_25*RLcp1_36-RLcp1_26*qd[4]
    ORcp1_26 = -OMcp1_15*RLcp1_36+RLcp1_16*qd[4]
    ORcp1_36 = OMcp1_15*RLcp1_26-OMcp1_25*RLcp1_16
    VIcp1_16 = ORcp1_16+VIcp1_15
    VIcp1_26 = ORcp1_26+VIcp1_25
    VIcp1_36 = ORcp1_36+qd[3]
    OPcp1_16 = OPcp1_15-qdd[6]*S4-qd[4]*qd[6]*C4
    OPcp1_26 = OPcp1_25+qdd[6]*C4-qd[4]*qd[6]*S4
    OPcp1_36 = qdd[4]+qd[6]*(OMcp1_15*C4+OMcp1_25*S4)
    ACcp1_16 = ACcp1_15+OMcp1_25*ORcp1_36+OPcp1_25*RLcp1_36-ORcp1_26*qd[4]-RLcp1_26*qdd[4]
    ACcp1_26 = ACcp1_25-OMcp1_15*ORcp1_36-OPcp1_15*RLcp1_36+ORcp1_16*qd[4]+RLcp1_16*qdd[4]
    ACcp1_36 = qdd[3]+OMcp1_15*ORcp1_26-OMcp1_25*ORcp1_16+OPcp1_15*RLcp1_26-OPcp1_25*RLcp1_16
    RLcp1_17 = ROcp1_76*s.dpt[3,3]
    RLcp1_27 = ROcp1_86*s.dpt[3,3]
    RLcp1_37 = ROcp1_96*s.dpt[3,3]
    POcp1_17 = POcp1_16+RLcp1_17
    POcp1_27 = POcp1_26+RLcp1_27
    POcp1_37 = POcp1_36+RLcp1_37
    OMcp1_17 = OMcp1_16+ROcp1_76*qd[7]
    OMcp1_27 = OMcp1_26+ROcp1_86*qd[7]
    OMcp1_37 = qd[4]+ROcp1_96*qd[7]
    ORcp1_17 = OMcp1_26*RLcp1_37-RLcp1_27*qd[4]
    ORcp1_27 = -OMcp1_16*RLcp1_37+RLcp1_17*qd[4]
    ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17
    VIcp1_17 = ORcp1_17+VIcp1_16
    VIcp1_27 = ORcp1_27+VIcp1_26
    VIcp1_37 = ORcp1_37+VIcp1_36
    OPcp1_17 = OPcp1_16+ROcp1_76*qdd[7]+qd[7]*(OMcp1_26*ROcp1_96-ROcp1_86*qd[4])
    OPcp1_27 = OPcp1_26+ROcp1_86*qdd[7]+qd[7]*(-OMcp1_16*ROcp1_96+ROcp1_76*qd[4])
    OPcp1_37 = OPcp1_36+ROcp1_96*qdd[7]+qd[7]*(OMcp1_16*ROcp1_86-OMcp1_26*ROcp1_76)
    ACcp1_17 = ACcp1_16+OMcp1_26*ORcp1_37+OPcp1_26*RLcp1_37-OPcp1_36*RLcp1_27-ORcp1_27*qd[4]
    ACcp1_27 = ACcp1_26-OMcp1_16*ORcp1_37-OPcp1_16*RLcp1_37+OPcp1_36*RLcp1_17+ORcp1_17*qd[4]
    ACcp1_37 = ACcp1_36+OMcp1_16*ORcp1_27-OMcp1_26*ORcp1_17+OPcp1_16*RLcp1_27-OPcp1_26*RLcp1_17
    OMcp1_18 = OMcp1_17+ROcp1_47*qd[8]
    OMcp1_28 = OMcp1_27+ROcp1_57*qd[8]
    OMcp1_38 = OMcp1_37+ROcp1_67*qd[8]
    OPcp1_18 = OPcp1_17+ROcp1_47*qdd[8]+qd[8]*(OMcp1_27*ROcp1_67-OMcp1_37*ROcp1_57)
    OPcp1_28 = OPcp1_27+ROcp1_57*qdd[8]+qd[8]*(-OMcp1_17*ROcp1_67+OMcp1_37*ROcp1_47)
    OPcp1_38 = OPcp1_37+ROcp1_67*qdd[8]+qd[8]*(OMcp1_17*ROcp1_57-OMcp1_27*ROcp1_47)
    RLcp1_19 = ROcp1_78*s.dpt[3,5]
    RLcp1_29 = ROcp1_88*s.dpt[3,5]
    RLcp1_39 = ROcp1_98*s.dpt[3,5]
    POcp1_19 = POcp1_17+RLcp1_19
    POcp1_29 = POcp1_27+RLcp1_29
    POcp1_39 = POcp1_37+RLcp1_39
    OMcp1_19 = OMcp1_18+ROcp1_47*qd[9]
    OMcp1_29 = OMcp1_28+ROcp1_57*qd[9]
    OMcp1_39 = OMcp1_38+ROcp1_67*qd[9]
    ORcp1_19 = OMcp1_28*RLcp1_39-OMcp1_38*RLcp1_29
    ORcp1_29 = -OMcp1_18*RLcp1_39+OMcp1_38*RLcp1_19
    ORcp1_39 = OMcp1_18*RLcp1_29-OMcp1_28*RLcp1_19
    VIcp1_19 = ORcp1_19+VIcp1_17
    VIcp1_29 = ORcp1_29+VIcp1_27
    VIcp1_39 = ORcp1_39+VIcp1_37
    OPcp1_19 = OPcp1_18+ROcp1_47*qdd[9]+qd[9]*(OMcp1_28*ROcp1_67-OMcp1_38*ROcp1_57)
    OPcp1_29 = OPcp1_28+ROcp1_57*qdd[9]+qd[9]*(-OMcp1_18*ROcp1_67+OMcp1_38*ROcp1_47)
    OPcp1_39 = OPcp1_38+ROcp1_67*qdd[9]+qd[9]*(OMcp1_18*ROcp1_57-OMcp1_28*ROcp1_47)
    ACcp1_19 = ACcp1_17+OMcp1_28*ORcp1_39-OMcp1_38*ORcp1_29+OPcp1_28*RLcp1_39-OPcp1_38*RLcp1_29
    ACcp1_29 = ACcp1_27-OMcp1_18*ORcp1_39+OMcp1_38*ORcp1_19-OPcp1_18*RLcp1_39+OPcp1_38*RLcp1_19
    ACcp1_39 = ACcp1_37+OMcp1_18*ORcp1_29-OMcp1_28*ORcp1_19+OPcp1_18*RLcp1_29-OPcp1_28*RLcp1_19
    RLcp1_110 = ROcp1_19*s.dpt[1,6]+ROcp1_47*s.dpt[2,6]+ROcp1_79*s.dpt[3,6]
    RLcp1_210 = ROcp1_29*s.dpt[1,6]+ROcp1_57*s.dpt[2,6]+ROcp1_89*s.dpt[3,6]
    RLcp1_310 = ROcp1_39*s.dpt[1,6]+ROcp1_67*s.dpt[2,6]+ROcp1_99*s.dpt[3,6]
    POcp1_110 = POcp1_19+RLcp1_110
    POcp1_210 = POcp1_29+RLcp1_210
    POcp1_310 = POcp1_39+RLcp1_310
    OMcp1_110 = OMcp1_19+ROcp1_47*qd[10]
    OMcp1_210 = OMcp1_29+ROcp1_57*qd[10]
    OMcp1_310 = OMcp1_39+ROcp1_67*qd[10]
    ORcp1_110 = OMcp1_29*RLcp1_310-OMcp1_39*RLcp1_210
    ORcp1_210 = -OMcp1_19*RLcp1_310+OMcp1_39*RLcp1_110
    ORcp1_310 = OMcp1_19*RLcp1_210-OMcp1_29*RLcp1_110
    VIcp1_110 = ORcp1_110+VIcp1_19
    VIcp1_210 = ORcp1_210+VIcp1_29
    VIcp1_310 = ORcp1_310+VIcp1_39
    OPcp1_110 = OPcp1_19+ROcp1_47*qdd[10]+qd[10]*(OMcp1_29*ROcp1_67-OMcp1_39*ROcp1_57)
    OPcp1_210 = OPcp1_29+ROcp1_57*qdd[10]+qd[10]*(-OMcp1_19*ROcp1_67+OMcp1_39*ROcp1_47)
    OPcp1_310 = OPcp1_39+ROcp1_67*qdd[10]+qd[10]*(OMcp1_19*ROcp1_57-OMcp1_29*ROcp1_47)
    ACcp1_110 = ACcp1_19+OMcp1_29*ORcp1_310-OMcp1_39*ORcp1_210+OPcp1_29*RLcp1_310-OPcp1_39*RLcp1_210
    ACcp1_210 = ACcp1_29-OMcp1_19*ORcp1_310+OMcp1_39*ORcp1_110-OPcp1_19*RLcp1_310+OPcp1_39*RLcp1_110
    ACcp1_310 = ACcp1_39+OMcp1_19*ORcp1_210-OMcp1_29*ORcp1_110+OPcp1_19*RLcp1_210-OPcp1_29*RLcp1_110
    RLcp1_111 = ROcp1_110*s.dpt[1,11]
    RLcp1_211 = ROcp1_210*s.dpt[1,11]
    RLcp1_311 = ROcp1_310*s.dpt[1,11]
    POcp1_111 = POcp1_110+RLcp1_111
    POcp1_211 = POcp1_210+RLcp1_211
    POcp1_311 = POcp1_310+RLcp1_311
    OMcp1_111 = OMcp1_110+ROcp1_47*qd[11]
    OMcp1_211 = OMcp1_210+ROcp1_57*qd[11]
    OMcp1_311 = OMcp1_310+ROcp1_67*qd[11]
    ORcp1_111 = OMcp1_210*RLcp1_311-OMcp1_310*RLcp1_211
    ORcp1_211 = -OMcp1_110*RLcp1_311+OMcp1_310*RLcp1_111
    ORcp1_311 = OMcp1_110*RLcp1_211-OMcp1_210*RLcp1_111
    VIcp1_111 = ORcp1_111+VIcp1_110
    VIcp1_211 = ORcp1_211+VIcp1_210
    VIcp1_311 = ORcp1_311+VIcp1_310
    OPcp1_111 = OPcp1_110+ROcp1_47*qdd[11]+qd[11]*(OMcp1_210*ROcp1_67-OMcp1_310*ROcp1_57)
    OPcp1_211 = OPcp1_210+ROcp1_57*qdd[11]+qd[11]*(-OMcp1_110*ROcp1_67+OMcp1_310*ROcp1_47)
    OPcp1_311 = OPcp1_310+ROcp1_67*qdd[11]+qd[11]*(OMcp1_110*ROcp1_57-OMcp1_210*ROcp1_47)
    ACcp1_111 = ACcp1_110+OMcp1_210*ORcp1_311-OMcp1_310*ORcp1_211+OPcp1_210*RLcp1_311-OPcp1_310*RLcp1_211
    ACcp1_211 = ACcp1_210-OMcp1_110*ORcp1_311+OMcp1_310*ORcp1_111-OPcp1_110*RLcp1_311+OPcp1_310*RLcp1_111
    ACcp1_311 = ACcp1_310+OMcp1_110*ORcp1_211-OMcp1_210*ORcp1_111+OPcp1_110*RLcp1_211-OPcp1_210*RLcp1_111
    sens.P[1] = POcp1_111
    sens.P[2] = POcp1_211
    sens.P[3] = POcp1_311
    sens.R[1,1] = ROcp1_111
    sens.R[1,2] = ROcp1_211
    sens.R[1,3] = ROcp1_311
    sens.R[2,1] = ROcp1_47
    sens.R[2,2] = ROcp1_57
    sens.R[2,3] = ROcp1_67
    sens.R[3,1] = ROcp1_711
    sens.R[3,2] = ROcp1_811
    sens.R[3,3] = ROcp1_911
    sens.V[1] = VIcp1_111
    sens.V[2] = VIcp1_211
    sens.V[3] = VIcp1_311
    sens.OM[1] = OMcp1_111
    sens.OM[2] = OMcp1_211
    sens.OM[3] = OMcp1_311
    sens.A[1] = ACcp1_111
    sens.A[2] = ACcp1_211
    sens.A[3] = ACcp1_311
    sens.OMP[1] = OPcp1_111
    sens.OMP[2] = OPcp1_211
    sens.OMP[3] = OPcp1_311

  if (isens == 2): 

    ROcp2_15 = C4*C5
    ROcp2_25 = S4*C5
    ROcp2_75 = C4*S5
    ROcp2_85 = S4*S5
    ROcp2_16 = ROcp2_15*C6-ROcp2_75*S6
    ROcp2_26 = ROcp2_25*C6-ROcp2_85*S6
    ROcp2_36 = -C5*S6-S5*C6
    ROcp2_76 = ROcp2_15*S6+ROcp2_75*C6
    ROcp2_86 = ROcp2_25*S6+ROcp2_85*C6
    ROcp2_96 = C5*C6-S5*S6
    ROcp2_17 = ROcp2_16*C7-S4*S7
    ROcp2_27 = ROcp2_26*C7+C4*S7
    ROcp2_37 = ROcp2_36*C7
    ROcp2_47 = -ROcp2_16*S7-S4*C7
    ROcp2_57 = -ROcp2_26*S7+C4*C7
    ROcp2_67 = -ROcp2_36*S7
    ROcp2_18 = ROcp2_17*C8-ROcp2_76*S8
    ROcp2_28 = ROcp2_27*C8-ROcp2_86*S8
    ROcp2_38 = ROcp2_37*C8-ROcp2_96*S8
    ROcp2_78 = ROcp2_17*S8+ROcp2_76*C8
    ROcp2_88 = ROcp2_27*S8+ROcp2_86*C8
    ROcp2_98 = ROcp2_37*S8+ROcp2_96*C8
    ROcp2_19 = ROcp2_18*C9-ROcp2_78*S9
    ROcp2_29 = ROcp2_28*C9-ROcp2_88*S9
    ROcp2_39 = ROcp2_38*C9-ROcp2_98*S9
    ROcp2_79 = ROcp2_18*S9+ROcp2_78*C9
    ROcp2_89 = ROcp2_28*S9+ROcp2_88*C9
    ROcp2_99 = ROcp2_38*S9+ROcp2_98*C9
    ROcp2_112 = ROcp2_19*C12-ROcp2_79*S12
    ROcp2_212 = ROcp2_29*C12-ROcp2_89*S12
    ROcp2_312 = ROcp2_39*C12-ROcp2_99*S12
    ROcp2_712 = ROcp2_19*S12+ROcp2_79*C12
    ROcp2_812 = ROcp2_29*S12+ROcp2_89*C12
    ROcp2_912 = ROcp2_39*S12+ROcp2_99*C12
    ROcp2_113 = ROcp2_112*C13-ROcp2_712*S13
    ROcp2_213 = ROcp2_212*C13-ROcp2_812*S13
    ROcp2_313 = ROcp2_312*C13-ROcp2_912*S13
    ROcp2_713 = ROcp2_112*S13+ROcp2_712*C13
    ROcp2_813 = ROcp2_212*S13+ROcp2_812*C13
    ROcp2_913 = ROcp2_312*S13+ROcp2_912*C13
    RLcp2_15 = s.dpt[1,1]*C4
    RLcp2_25 = s.dpt[1,1]*S4
    POcp2_15 = RLcp2_15+q[1]
    POcp2_25 = RLcp2_25+q[2]
    POcp2_35 = q[3]+s.dpt[3,1]
    OMcp2_15 = -qd[5]*S4
    OMcp2_25 = qd[5]*C4
    ORcp2_15 = -RLcp2_25*qd[4]
    ORcp2_25 = RLcp2_15*qd[4]
    VIcp2_15 = ORcp2_15+qd[1]
    VIcp2_25 = ORcp2_25+qd[2]
    OPcp2_15 = -qdd[5]*S4-qd[4]*qd[5]*C4
    OPcp2_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    ACcp2_15 = qdd[1]-ORcp2_25*qd[4]-RLcp2_25*qdd[4]
    ACcp2_25 = qdd[2]+ORcp2_15*qd[4]+RLcp2_15*qdd[4]
    RLcp2_16 = ROcp2_15*s.dpt[1,2]
    RLcp2_26 = ROcp2_25*s.dpt[1,2]
    RLcp2_36 = -s.dpt[1,2]*S5
    POcp2_16 = POcp2_15+RLcp2_16
    POcp2_26 = POcp2_25+RLcp2_26
    POcp2_36 = POcp2_35+RLcp2_36
    OMcp2_16 = OMcp2_15-qd[6]*S4
    OMcp2_26 = OMcp2_25+qd[6]*C4
    ORcp2_16 = OMcp2_25*RLcp2_36-RLcp2_26*qd[4]
    ORcp2_26 = -OMcp2_15*RLcp2_36+RLcp2_16*qd[4]
    ORcp2_36 = OMcp2_15*RLcp2_26-OMcp2_25*RLcp2_16
    VIcp2_16 = ORcp2_16+VIcp2_15
    VIcp2_26 = ORcp2_26+VIcp2_25
    VIcp2_36 = ORcp2_36+qd[3]
    OPcp2_16 = OPcp2_15-qdd[6]*S4-qd[4]*qd[6]*C4
    OPcp2_26 = OPcp2_25+qdd[6]*C4-qd[4]*qd[6]*S4
    OPcp2_36 = qdd[4]+qd[6]*(OMcp2_15*C4+OMcp2_25*S4)
    ACcp2_16 = ACcp2_15+OMcp2_25*ORcp2_36+OPcp2_25*RLcp2_36-ORcp2_26*qd[4]-RLcp2_26*qdd[4]
    ACcp2_26 = ACcp2_25-OMcp2_15*ORcp2_36-OPcp2_15*RLcp2_36+ORcp2_16*qd[4]+RLcp2_16*qdd[4]
    ACcp2_36 = qdd[3]+OMcp2_15*ORcp2_26-OMcp2_25*ORcp2_16+OPcp2_15*RLcp2_26-OPcp2_25*RLcp2_16
    RLcp2_17 = ROcp2_76*s.dpt[3,3]
    RLcp2_27 = ROcp2_86*s.dpt[3,3]
    RLcp2_37 = ROcp2_96*s.dpt[3,3]
    POcp2_17 = POcp2_16+RLcp2_17
    POcp2_27 = POcp2_26+RLcp2_27
    POcp2_37 = POcp2_36+RLcp2_37
    OMcp2_17 = OMcp2_16+ROcp2_76*qd[7]
    OMcp2_27 = OMcp2_26+ROcp2_86*qd[7]
    OMcp2_37 = qd[4]+ROcp2_96*qd[7]
    ORcp2_17 = OMcp2_26*RLcp2_37-RLcp2_27*qd[4]
    ORcp2_27 = -OMcp2_16*RLcp2_37+RLcp2_17*qd[4]
    ORcp2_37 = OMcp2_16*RLcp2_27-OMcp2_26*RLcp2_17
    VIcp2_17 = ORcp2_17+VIcp2_16
    VIcp2_27 = ORcp2_27+VIcp2_26
    VIcp2_37 = ORcp2_37+VIcp2_36
    OPcp2_17 = OPcp2_16+ROcp2_76*qdd[7]+qd[7]*(OMcp2_26*ROcp2_96-ROcp2_86*qd[4])
    OPcp2_27 = OPcp2_26+ROcp2_86*qdd[7]+qd[7]*(-OMcp2_16*ROcp2_96+ROcp2_76*qd[4])
    OPcp2_37 = OPcp2_36+ROcp2_96*qdd[7]+qd[7]*(OMcp2_16*ROcp2_86-OMcp2_26*ROcp2_76)
    ACcp2_17 = ACcp2_16+OMcp2_26*ORcp2_37+OPcp2_26*RLcp2_37-OPcp2_36*RLcp2_27-ORcp2_27*qd[4]
    ACcp2_27 = ACcp2_26-OMcp2_16*ORcp2_37-OPcp2_16*RLcp2_37+OPcp2_36*RLcp2_17+ORcp2_17*qd[4]
    ACcp2_37 = ACcp2_36+OMcp2_16*ORcp2_27-OMcp2_26*ORcp2_17+OPcp2_16*RLcp2_27-OPcp2_26*RLcp2_17
    OMcp2_18 = OMcp2_17+ROcp2_47*qd[8]
    OMcp2_28 = OMcp2_27+ROcp2_57*qd[8]
    OMcp2_38 = OMcp2_37+ROcp2_67*qd[8]
    OPcp2_18 = OPcp2_17+ROcp2_47*qdd[8]+qd[8]*(OMcp2_27*ROcp2_67-OMcp2_37*ROcp2_57)
    OPcp2_28 = OPcp2_27+ROcp2_57*qdd[8]+qd[8]*(-OMcp2_17*ROcp2_67+OMcp2_37*ROcp2_47)
    OPcp2_38 = OPcp2_37+ROcp2_67*qdd[8]+qd[8]*(OMcp2_17*ROcp2_57-OMcp2_27*ROcp2_47)
    RLcp2_19 = ROcp2_78*s.dpt[3,5]
    RLcp2_29 = ROcp2_88*s.dpt[3,5]
    RLcp2_39 = ROcp2_98*s.dpt[3,5]
    POcp2_19 = POcp2_17+RLcp2_19
    POcp2_29 = POcp2_27+RLcp2_29
    POcp2_39 = POcp2_37+RLcp2_39
    OMcp2_19 = OMcp2_18+ROcp2_47*qd[9]
    OMcp2_29 = OMcp2_28+ROcp2_57*qd[9]
    OMcp2_39 = OMcp2_38+ROcp2_67*qd[9]
    ORcp2_19 = OMcp2_28*RLcp2_39-OMcp2_38*RLcp2_29
    ORcp2_29 = -OMcp2_18*RLcp2_39+OMcp2_38*RLcp2_19
    ORcp2_39 = OMcp2_18*RLcp2_29-OMcp2_28*RLcp2_19
    VIcp2_19 = ORcp2_19+VIcp2_17
    VIcp2_29 = ORcp2_29+VIcp2_27
    VIcp2_39 = ORcp2_39+VIcp2_37
    OPcp2_19 = OPcp2_18+ROcp2_47*qdd[9]+qd[9]*(OMcp2_28*ROcp2_67-OMcp2_38*ROcp2_57)
    OPcp2_29 = OPcp2_28+ROcp2_57*qdd[9]+qd[9]*(-OMcp2_18*ROcp2_67+OMcp2_38*ROcp2_47)
    OPcp2_39 = OPcp2_38+ROcp2_67*qdd[9]+qd[9]*(OMcp2_18*ROcp2_57-OMcp2_28*ROcp2_47)
    ACcp2_19 = ACcp2_17+OMcp2_28*ORcp2_39-OMcp2_38*ORcp2_29+OPcp2_28*RLcp2_39-OPcp2_38*RLcp2_29
    ACcp2_29 = ACcp2_27-OMcp2_18*ORcp2_39+OMcp2_38*ORcp2_19-OPcp2_18*RLcp2_39+OPcp2_38*RLcp2_19
    ACcp2_39 = ACcp2_37+OMcp2_18*ORcp2_29-OMcp2_28*ORcp2_19+OPcp2_18*RLcp2_29-OPcp2_28*RLcp2_19
    RLcp2_110 = ROcp2_19*s.dpt[1,7]+ROcp2_47*s.dpt[2,7]+ROcp2_79*s.dpt[3,7]
    RLcp2_210 = ROcp2_29*s.dpt[1,7]+ROcp2_57*s.dpt[2,7]+ROcp2_89*s.dpt[3,7]
    RLcp2_310 = ROcp2_39*s.dpt[1,7]+ROcp2_67*s.dpt[2,7]+ROcp2_99*s.dpt[3,7]
    POcp2_110 = POcp2_19+RLcp2_110
    POcp2_210 = POcp2_29+RLcp2_210
    POcp2_310 = POcp2_39+RLcp2_310
    OMcp2_110 = OMcp2_19+ROcp2_47*qd[12]
    OMcp2_210 = OMcp2_29+ROcp2_57*qd[12]
    OMcp2_310 = OMcp2_39+ROcp2_67*qd[12]
    ORcp2_110 = OMcp2_29*RLcp2_310-OMcp2_39*RLcp2_210
    ORcp2_210 = -OMcp2_19*RLcp2_310+OMcp2_39*RLcp2_110
    ORcp2_310 = OMcp2_19*RLcp2_210-OMcp2_29*RLcp2_110
    VIcp2_110 = ORcp2_110+VIcp2_19
    VIcp2_210 = ORcp2_210+VIcp2_29
    VIcp2_310 = ORcp2_310+VIcp2_39
    OPcp2_110 = OPcp2_19+ROcp2_47*qdd[12]+qd[12]*(OMcp2_29*ROcp2_67-OMcp2_39*ROcp2_57)
    OPcp2_210 = OPcp2_29+ROcp2_57*qdd[12]+qd[12]*(-OMcp2_19*ROcp2_67+OMcp2_39*ROcp2_47)
    OPcp2_310 = OPcp2_39+ROcp2_67*qdd[12]+qd[12]*(OMcp2_19*ROcp2_57-OMcp2_29*ROcp2_47)
    ACcp2_110 = ACcp2_19+OMcp2_29*ORcp2_310-OMcp2_39*ORcp2_210+OPcp2_29*RLcp2_310-OPcp2_39*RLcp2_210
    ACcp2_210 = ACcp2_29-OMcp2_19*ORcp2_310+OMcp2_39*ORcp2_110-OPcp2_19*RLcp2_310+OPcp2_39*RLcp2_110
    ACcp2_310 = ACcp2_39+OMcp2_19*ORcp2_210-OMcp2_29*ORcp2_110+OPcp2_19*RLcp2_210-OPcp2_29*RLcp2_110
    RLcp2_111 = ROcp2_112*s.dpt[1,14]
    RLcp2_211 = ROcp2_212*s.dpt[1,14]
    RLcp2_311 = ROcp2_312*s.dpt[1,14]
    POcp2_111 = POcp2_110+RLcp2_111
    POcp2_211 = POcp2_210+RLcp2_211
    POcp2_311 = POcp2_310+RLcp2_311
    OMcp2_111 = OMcp2_110+ROcp2_47*qd[13]
    OMcp2_211 = OMcp2_210+ROcp2_57*qd[13]
    OMcp2_311 = OMcp2_310+ROcp2_67*qd[13]
    ORcp2_111 = OMcp2_210*RLcp2_311-OMcp2_310*RLcp2_211
    ORcp2_211 = -OMcp2_110*RLcp2_311+OMcp2_310*RLcp2_111
    ORcp2_311 = OMcp2_110*RLcp2_211-OMcp2_210*RLcp2_111
    VIcp2_111 = ORcp2_111+VIcp2_110
    VIcp2_211 = ORcp2_211+VIcp2_210
    VIcp2_311 = ORcp2_311+VIcp2_310
    OPcp2_111 = OPcp2_110+ROcp2_47*qdd[13]+qd[13]*(OMcp2_210*ROcp2_67-OMcp2_310*ROcp2_57)
    OPcp2_211 = OPcp2_210+ROcp2_57*qdd[13]+qd[13]*(-OMcp2_110*ROcp2_67+OMcp2_310*ROcp2_47)
    OPcp2_311 = OPcp2_310+ROcp2_67*qdd[13]+qd[13]*(OMcp2_110*ROcp2_57-OMcp2_210*ROcp2_47)
    ACcp2_111 = ACcp2_110+OMcp2_210*ORcp2_311-OMcp2_310*ORcp2_211+OPcp2_210*RLcp2_311-OPcp2_310*RLcp2_211
    ACcp2_211 = ACcp2_210-OMcp2_110*ORcp2_311+OMcp2_310*ORcp2_111-OPcp2_110*RLcp2_311+OPcp2_310*RLcp2_111
    ACcp2_311 = ACcp2_310+OMcp2_110*ORcp2_211-OMcp2_210*ORcp2_111+OPcp2_110*RLcp2_211-OPcp2_210*RLcp2_111
    sens.P[1] = POcp2_111
    sens.P[2] = POcp2_211
    sens.P[3] = POcp2_311
    sens.R[1,1] = ROcp2_113
    sens.R[1,2] = ROcp2_213
    sens.R[1,3] = ROcp2_313
    sens.R[2,1] = ROcp2_47
    sens.R[2,2] = ROcp2_57
    sens.R[2,3] = ROcp2_67
    sens.R[3,1] = ROcp2_713
    sens.R[3,2] = ROcp2_813
    sens.R[3,3] = ROcp2_913
    sens.V[1] = VIcp2_111
    sens.V[2] = VIcp2_211
    sens.V[3] = VIcp2_311
    sens.OM[1] = OMcp2_111
    sens.OM[2] = OMcp2_211
    sens.OM[3] = OMcp2_311
    sens.A[1] = ACcp2_111
    sens.A[2] = ACcp2_211
    sens.A[3] = ACcp2_311
    sens.OMP[1] = OPcp2_111
    sens.OMP[2] = OPcp2_211
    sens.OMP[3] = OPcp2_311

  if (isens == 3): 

    ROcp3_15 = C4*C5
    ROcp3_25 = S4*C5
    ROcp3_75 = C4*S5
    ROcp3_85 = S4*S5
    ROcp3_16 = ROcp3_15*C6-ROcp3_75*S6
    ROcp3_26 = ROcp3_25*C6-ROcp3_85*S6
    ROcp3_36 = -C5*S6-S5*C6
    ROcp3_76 = ROcp3_15*S6+ROcp3_75*C6
    ROcp3_86 = ROcp3_25*S6+ROcp3_85*C6
    ROcp3_96 = C5*C6-S5*S6
    ROcp3_116 = ROcp3_16*C16-ROcp3_76*S16
    ROcp3_216 = ROcp3_26*C16-ROcp3_86*S16
    ROcp3_316 = ROcp3_36*C16-ROcp3_96*S16
    ROcp3_716 = ROcp3_16*S16+ROcp3_76*C16
    ROcp3_816 = ROcp3_26*S16+ROcp3_86*C16
    ROcp3_916 = ROcp3_36*S16+ROcp3_96*C16
    RLcp3_15 = s.dpt[1,1]*C4
    RLcp3_25 = s.dpt[1,1]*S4
    POcp3_15 = RLcp3_15+q[1]
    POcp3_25 = RLcp3_25+q[2]
    POcp3_35 = q[3]+s.dpt[3,1]
    OMcp3_15 = -qd[5]*S4
    OMcp3_25 = qd[5]*C4
    ORcp3_15 = -RLcp3_25*qd[4]
    ORcp3_25 = RLcp3_15*qd[4]
    VIcp3_15 = ORcp3_15+qd[1]
    VIcp3_25 = ORcp3_25+qd[2]
    OPcp3_15 = -qdd[5]*S4-qd[4]*qd[5]*C4
    OPcp3_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    ACcp3_15 = qdd[1]-ORcp3_25*qd[4]-RLcp3_25*qdd[4]
    ACcp3_25 = qdd[2]+ORcp3_15*qd[4]+RLcp3_15*qdd[4]
    RLcp3_16 = ROcp3_15*s.dpt[1,2]
    RLcp3_26 = ROcp3_25*s.dpt[1,2]
    RLcp3_36 = -s.dpt[1,2]*S5
    POcp3_16 = POcp3_15+RLcp3_16
    POcp3_26 = POcp3_25+RLcp3_26
    POcp3_36 = POcp3_35+RLcp3_36
    OMcp3_16 = OMcp3_15-qd[6]*S4
    OMcp3_26 = OMcp3_25+qd[6]*C4
    ORcp3_16 = OMcp3_25*RLcp3_36-RLcp3_26*qd[4]
    ORcp3_26 = -OMcp3_15*RLcp3_36+RLcp3_16*qd[4]
    ORcp3_36 = OMcp3_15*RLcp3_26-OMcp3_25*RLcp3_16
    VIcp3_16 = ORcp3_16+VIcp3_15
    VIcp3_26 = ORcp3_26+VIcp3_25
    VIcp3_36 = ORcp3_36+qd[3]
    OPcp3_16 = OPcp3_15-qdd[6]*S4-qd[4]*qd[6]*C4
    OPcp3_26 = OPcp3_25+qdd[6]*C4-qd[4]*qd[6]*S4
    OPcp3_36 = qdd[4]+qd[6]*(OMcp3_15*C4+OMcp3_25*S4)
    ACcp3_16 = ACcp3_15+OMcp3_25*ORcp3_36+OPcp3_25*RLcp3_36-ORcp3_26*qd[4]-RLcp3_26*qdd[4]
    ACcp3_26 = ACcp3_25-OMcp3_15*ORcp3_36-OPcp3_15*RLcp3_36+ORcp3_16*qd[4]+RLcp3_16*qdd[4]
    ACcp3_36 = qdd[3]+OMcp3_15*ORcp3_26-OMcp3_25*ORcp3_16+OPcp3_15*RLcp3_26-OPcp3_25*RLcp3_16
    RLcp3_17 = ROcp3_76*Dz153
    RLcp3_27 = ROcp3_86*Dz153
    RLcp3_37 = ROcp3_96*Dz153
    POcp3_17 = POcp3_16+RLcp3_17
    POcp3_27 = POcp3_26+RLcp3_27
    POcp3_37 = POcp3_36+RLcp3_37
    ORcp3_17 = OMcp3_26*RLcp3_37-RLcp3_27*qd[4]
    ORcp3_27 = -OMcp3_16*RLcp3_37+RLcp3_17*qd[4]
    ORcp3_37 = OMcp3_16*RLcp3_27-OMcp3_26*RLcp3_17
    VIcp3_17 = ORcp3_17+VIcp3_16+ROcp3_76*qd[15]
    VIcp3_27 = ORcp3_27+VIcp3_26+ROcp3_86*qd[15]
    VIcp3_37 = ORcp3_37+VIcp3_36+ROcp3_96*qd[15]
    ACcp3_17 = ACcp3_16+OMcp3_26*ORcp3_37+OPcp3_26*RLcp3_37-OPcp3_36*RLcp3_27-ORcp3_27*qd[4]+ROcp3_76*qdd[15]+(2.0)*qd[15]*(OMcp3_26*ROcp3_96-ROcp3_86*qd[4])
    ACcp3_27 = ACcp3_26-OMcp3_16*ORcp3_37-OPcp3_16*RLcp3_37+OPcp3_36*RLcp3_17+ORcp3_17*qd[4]+ROcp3_86*qdd[15]+(2.0)*qd[15]*(-OMcp3_16*ROcp3_96+ROcp3_76*qd[4])
    ACcp3_37 = ACcp3_36+OMcp3_16*ORcp3_27-OMcp3_26*ORcp3_17+OPcp3_16*RLcp3_27-OPcp3_26*RLcp3_17+ROcp3_96*qdd[15]+(2.0)*qd[15]*(OMcp3_16*ROcp3_86-OMcp3_26*ROcp3_76)
    RLcp3_18 = ROcp3_76*s.dpt[3,17]
    RLcp3_28 = ROcp3_86*s.dpt[3,17]
    RLcp3_38 = ROcp3_96*s.dpt[3,17]
    POcp3_18 = POcp3_17+RLcp3_18
    POcp3_28 = POcp3_27+RLcp3_28
    POcp3_38 = POcp3_37+RLcp3_38
    OMcp3_18 = OMcp3_16-qd[16]*S4
    OMcp3_28 = OMcp3_26+qd[16]*C4
    ORcp3_18 = OMcp3_26*RLcp3_38-RLcp3_28*qd[4]
    ORcp3_28 = -OMcp3_16*RLcp3_38+RLcp3_18*qd[4]
    ORcp3_38 = OMcp3_16*RLcp3_28-OMcp3_26*RLcp3_18
    VIcp3_18 = ORcp3_18+VIcp3_17
    VIcp3_28 = ORcp3_28+VIcp3_27
    VIcp3_38 = ORcp3_38+VIcp3_37
    OPcp3_18 = OPcp3_16-qdd[16]*S4-qd[16]*qd[4]*C4
    OPcp3_28 = OPcp3_26+qdd[16]*C4-qd[16]*qd[4]*S4
    OPcp3_38 = OPcp3_36+qd[16]*(OMcp3_16*C4+OMcp3_26*S4)
    ACcp3_18 = ACcp3_17+OMcp3_26*ORcp3_38+OPcp3_26*RLcp3_38-OPcp3_36*RLcp3_28-ORcp3_28*qd[4]
    ACcp3_28 = ACcp3_27-OMcp3_16*ORcp3_38-OPcp3_16*RLcp3_38+OPcp3_36*RLcp3_18+ORcp3_18*qd[4]
    ACcp3_38 = ACcp3_37+OMcp3_16*ORcp3_28-OMcp3_26*ORcp3_18+OPcp3_16*RLcp3_28-OPcp3_26*RLcp3_18
    sens.P[1] = POcp3_18
    sens.P[2] = POcp3_28
    sens.P[3] = POcp3_38
    sens.R[1,1] = ROcp3_116
    sens.R[1,2] = ROcp3_216
    sens.R[1,3] = ROcp3_316
    sens.R[2,1] = -S4
    sens.R[2,2] = C4
    sens.R[3,1] = ROcp3_716
    sens.R[3,2] = ROcp3_816
    sens.R[3,3] = ROcp3_916
    sens.V[1] = VIcp3_18
    sens.V[2] = VIcp3_28
    sens.V[3] = VIcp3_38
    sens.OM[1] = OMcp3_18
    sens.OM[2] = OMcp3_28
    sens.OM[3] = qd[4]
    sens.A[1] = ACcp3_18
    sens.A[2] = ACcp3_28
    sens.A[3] = ACcp3_38
    sens.OMP[1] = OPcp3_18
    sens.OMP[2] = OPcp3_28
    sens.OMP[3] = OPcp3_38

 


# Number of continuation lines = 0


