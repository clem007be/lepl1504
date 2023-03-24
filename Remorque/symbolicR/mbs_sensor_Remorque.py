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
#	==> Generation Date: Fri Mar 24 01:00:00 2023
#
#	==> Project name: Remorque
#
#	==> Number of joints: 22
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
  S11 = sin(q[11])
  C11 = cos(q[11])
  S12 = sin(q[12])
  C12 = cos(q[12])
  S13 = sin(q[13])
  C13 = cos(q[13])
  S15 = sin(q[15])
  C15 = cos(q[15])
  S17 = sin(q[17])
  C17 = cos(q[17])
  S18 = sin(q[18])
  C18 = cos(q[18])
  S19 = sin(q[19])
  C19 = cos(q[19])
  S20 = sin(q[20])
  C20 = cos(q[20])
  S21 = sin(q[21])
  C21 = cos(q[21])
  S22 = sin(q[22])
  C22 = cos(q[22])
 
# Augmented Joint Position Vectors

  Dz101 = q[10]+s.dpt[1,4]
  Dz143 = q[14]+s.dpt[3,8]
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

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
    ROcp1_111 = ROcp1_19*C11-ROcp1_79*S11
    ROcp1_211 = ROcp1_29*C11-ROcp1_89*S11
    ROcp1_311 = ROcp1_39*C11-ROcp1_99*S11
    ROcp1_711 = ROcp1_19*S11+ROcp1_79*C11
    ROcp1_811 = ROcp1_29*S11+ROcp1_89*C11
    ROcp1_911 = ROcp1_39*S11+ROcp1_99*C11
    ROcp1_112 = ROcp1_111*C12-ROcp1_711*S12
    ROcp1_212 = ROcp1_211*C12-ROcp1_811*S12
    ROcp1_312 = ROcp1_311*C12-ROcp1_911*S12
    ROcp1_712 = ROcp1_111*S12+ROcp1_711*C12
    ROcp1_812 = ROcp1_211*S12+ROcp1_811*C12
    ROcp1_912 = ROcp1_311*S12+ROcp1_911*C12
    ROcp1_113 = ROcp1_112*C13+ROcp1_45*S13
    ROcp1_213 = ROcp1_212*C13+ROcp1_55*S13
    ROcp1_313 = ROcp1_312*C13+S13*S5
    ROcp1_413 = -ROcp1_112*S13+ROcp1_45*C13
    ROcp1_513 = -ROcp1_212*S13+ROcp1_55*C13
    ROcp1_613 = -ROcp1_312*S13+C13*S5
    ROcp1_115 = ROcp1_113*C15-ROcp1_712*S15
    ROcp1_215 = ROcp1_213*C15-ROcp1_812*S15
    ROcp1_315 = ROcp1_313*C15-ROcp1_912*S15
    ROcp1_715 = ROcp1_113*S15+ROcp1_712*C15
    ROcp1_815 = ROcp1_213*S15+ROcp1_812*C15
    ROcp1_915 = ROcp1_313*S15+ROcp1_912*C15
    OMcp1_15 = qd[5]*C4
    OMcp1_25 = qd[5]*S4
    OPcp1_15 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp1_25 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp1_16 = OMcp1_15+ROcp1_45*qd[6]
    OMcp1_26 = OMcp1_25+ROcp1_55*qd[6]
    OMcp1_36 = qd[4]+qd[6]*S5
    OPcp1_16 = OPcp1_15+ROcp1_45*qdd[6]+qd[6]*(OMcp1_25*S5-ROcp1_55*qd[4])
    OPcp1_26 = OPcp1_25+ROcp1_55*qdd[6]+qd[6]*(-OMcp1_15*S5+ROcp1_45*qd[4])
    OPcp1_36 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp1_15*ROcp1_55-OMcp1_25*ROcp1_45)
    RLcp1_17 = ROcp1_16*s.dpt[1,1]
    RLcp1_27 = ROcp1_26*s.dpt[1,1]
    RLcp1_37 = ROcp1_36*s.dpt[1,1]
    POcp1_17 = RLcp1_17+q[1]
    POcp1_27 = RLcp1_27+q[2]
    POcp1_37 = RLcp1_37+q[3]
    OMcp1_17 = OMcp1_16+ROcp1_45*qd[7]
    OMcp1_27 = OMcp1_26+ROcp1_55*qd[7]
    OMcp1_37 = OMcp1_36+qd[7]*S5
    ORcp1_17 = OMcp1_26*RLcp1_37-OMcp1_36*RLcp1_27
    ORcp1_27 = -OMcp1_16*RLcp1_37+OMcp1_36*RLcp1_17
    ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17
    VIcp1_17 = ORcp1_17+qd[1]
    VIcp1_27 = ORcp1_27+qd[2]
    VIcp1_37 = ORcp1_37+qd[3]
    OPcp1_17 = OPcp1_16+ROcp1_45*qdd[7]+qd[7]*(OMcp1_26*S5-OMcp1_36*ROcp1_55)
    OPcp1_27 = OPcp1_26+ROcp1_55*qdd[7]+qd[7]*(-OMcp1_16*S5+OMcp1_36*ROcp1_45)
    OPcp1_37 = OPcp1_36+qdd[7]*S5+qd[7]*(OMcp1_16*ROcp1_55-OMcp1_26*ROcp1_45)
    ACcp1_17 = qdd[1]+OMcp1_26*ORcp1_37-OMcp1_36*ORcp1_27+OPcp1_26*RLcp1_37-OPcp1_36*RLcp1_27
    ACcp1_27 = qdd[2]-OMcp1_16*ORcp1_37+OMcp1_36*ORcp1_17-OPcp1_16*RLcp1_37+OPcp1_36*RLcp1_17
    ACcp1_37 = qdd[3]+OMcp1_16*ORcp1_27-OMcp1_26*ORcp1_17+OPcp1_16*RLcp1_27-OPcp1_26*RLcp1_17
    RLcp1_18 = ROcp1_17*s.dpt[1,2]
    RLcp1_28 = ROcp1_27*s.dpt[1,2]
    RLcp1_38 = ROcp1_37*s.dpt[1,2]
    POcp1_18 = POcp1_17+RLcp1_18
    POcp1_28 = POcp1_27+RLcp1_28
    POcp1_38 = POcp1_37+RLcp1_38
    OMcp1_18 = OMcp1_17+ROcp1_45*qd[8]
    OMcp1_28 = OMcp1_27+ROcp1_55*qd[8]
    OMcp1_38 = OMcp1_37+qd[8]*S5
    ORcp1_18 = OMcp1_27*RLcp1_38-OMcp1_37*RLcp1_28
    ORcp1_28 = -OMcp1_17*RLcp1_38+OMcp1_37*RLcp1_18
    ORcp1_38 = OMcp1_17*RLcp1_28-OMcp1_27*RLcp1_18
    VIcp1_18 = ORcp1_18+VIcp1_17
    VIcp1_28 = ORcp1_28+VIcp1_27
    VIcp1_38 = ORcp1_38+VIcp1_37
    OPcp1_18 = OPcp1_17+ROcp1_45*qdd[8]+qd[8]*(OMcp1_27*S5-OMcp1_37*ROcp1_55)
    OPcp1_28 = OPcp1_27+ROcp1_55*qdd[8]+qd[8]*(-OMcp1_17*S5+OMcp1_37*ROcp1_45)
    OPcp1_38 = OPcp1_37+qdd[8]*S5+qd[8]*(OMcp1_17*ROcp1_55-OMcp1_27*ROcp1_45)
    ACcp1_18 = ACcp1_17+OMcp1_27*ORcp1_38-OMcp1_37*ORcp1_28+OPcp1_27*RLcp1_38-OPcp1_37*RLcp1_28
    ACcp1_28 = ACcp1_27-OMcp1_17*ORcp1_38+OMcp1_37*ORcp1_18-OPcp1_17*RLcp1_38+OPcp1_37*RLcp1_18
    ACcp1_38 = ACcp1_37+OMcp1_17*ORcp1_28-OMcp1_27*ORcp1_18+OPcp1_17*RLcp1_28-OPcp1_27*RLcp1_18
    RLcp1_19 = ROcp1_18*s.dpt[1,3]
    RLcp1_29 = ROcp1_28*s.dpt[1,3]
    RLcp1_39 = ROcp1_38*s.dpt[1,3]
    POcp1_19 = POcp1_18+RLcp1_19
    POcp1_29 = POcp1_28+RLcp1_29
    POcp1_39 = POcp1_38+RLcp1_39
    OMcp1_19 = OMcp1_18+ROcp1_45*qd[9]
    OMcp1_29 = OMcp1_28+ROcp1_55*qd[9]
    OMcp1_39 = OMcp1_38+qd[9]*S5
    ORcp1_19 = OMcp1_28*RLcp1_39-OMcp1_38*RLcp1_29
    ORcp1_29 = -OMcp1_18*RLcp1_39+OMcp1_38*RLcp1_19
    ORcp1_39 = OMcp1_18*RLcp1_29-OMcp1_28*RLcp1_19
    VIcp1_19 = ORcp1_19+VIcp1_18
    VIcp1_29 = ORcp1_29+VIcp1_28
    VIcp1_39 = ORcp1_39+VIcp1_38
    OPcp1_19 = OPcp1_18+ROcp1_45*qdd[9]+qd[9]*(OMcp1_28*S5-OMcp1_38*ROcp1_55)
    OPcp1_29 = OPcp1_28+ROcp1_55*qdd[9]+qd[9]*(-OMcp1_18*S5+OMcp1_38*ROcp1_45)
    OPcp1_39 = OPcp1_38+qdd[9]*S5+qd[9]*(OMcp1_18*ROcp1_55-OMcp1_28*ROcp1_45)
    ACcp1_19 = ACcp1_18+OMcp1_28*ORcp1_39-OMcp1_38*ORcp1_29+OPcp1_28*RLcp1_39-OPcp1_38*RLcp1_29
    ACcp1_29 = ACcp1_28-OMcp1_18*ORcp1_39+OMcp1_38*ORcp1_19-OPcp1_18*RLcp1_39+OPcp1_38*RLcp1_19
    ACcp1_39 = ACcp1_38+OMcp1_18*ORcp1_29-OMcp1_28*ORcp1_19+OPcp1_18*RLcp1_29-OPcp1_28*RLcp1_19
    RLcp1_110 = ROcp1_19*Dz101
    RLcp1_210 = ROcp1_29*Dz101
    RLcp1_310 = ROcp1_39*Dz101
    POcp1_110 = POcp1_19+RLcp1_110
    POcp1_210 = POcp1_29+RLcp1_210
    POcp1_310 = POcp1_39+RLcp1_310
    ORcp1_110 = OMcp1_29*RLcp1_310-OMcp1_39*RLcp1_210
    ORcp1_210 = -OMcp1_19*RLcp1_310+OMcp1_39*RLcp1_110
    ORcp1_310 = OMcp1_19*RLcp1_210-OMcp1_29*RLcp1_110
    VIcp1_110 = ORcp1_110+VIcp1_19+ROcp1_19*qd[10]
    VIcp1_210 = ORcp1_210+VIcp1_29+ROcp1_29*qd[10]
    VIcp1_310 = ORcp1_310+VIcp1_39+ROcp1_39*qd[10]
    ACcp1_110 = ACcp1_19+OMcp1_29*ORcp1_310-OMcp1_39*ORcp1_210+OPcp1_29*RLcp1_310-OPcp1_39*RLcp1_210+ROcp1_19*qdd[10]+(2.0)*qd[10]*(OMcp1_29*ROcp1_39-OMcp1_39*ROcp1_29)
    ACcp1_210 = ACcp1_29-OMcp1_19*ORcp1_310+OMcp1_39*ORcp1_110-OPcp1_19*RLcp1_310+OPcp1_39*RLcp1_110+ROcp1_29*qdd[10]+(2.0)*qd[10]*(-OMcp1_19*ROcp1_39+OMcp1_39*ROcp1_19)
    ACcp1_310 = ACcp1_39+OMcp1_19*ORcp1_210-OMcp1_29*ORcp1_110+OPcp1_19*RLcp1_210-OPcp1_29*RLcp1_110+ROcp1_39*qdd[10]+(2.0)*qd[10]*(OMcp1_19*ROcp1_29-OMcp1_29*ROcp1_19)
    RLcp1_111 = ROcp1_19*s.dpt[1,5]
    RLcp1_211 = ROcp1_29*s.dpt[1,5]
    RLcp1_311 = ROcp1_39*s.dpt[1,5]
    POcp1_111 = POcp1_110+RLcp1_111
    POcp1_211 = POcp1_210+RLcp1_211
    POcp1_311 = POcp1_310+RLcp1_311
    OMcp1_111 = OMcp1_19+ROcp1_45*qd[11]
    OMcp1_211 = OMcp1_29+ROcp1_55*qd[11]
    OMcp1_311 = OMcp1_39+qd[11]*S5
    ORcp1_111 = OMcp1_29*RLcp1_311-OMcp1_39*RLcp1_211
    ORcp1_211 = -OMcp1_19*RLcp1_311+OMcp1_39*RLcp1_111
    ORcp1_311 = OMcp1_19*RLcp1_211-OMcp1_29*RLcp1_111
    VIcp1_111 = ORcp1_111+VIcp1_110
    VIcp1_211 = ORcp1_211+VIcp1_210
    VIcp1_311 = ORcp1_311+VIcp1_310
    OPcp1_111 = OPcp1_19+ROcp1_45*qdd[11]+qd[11]*(OMcp1_29*S5-OMcp1_39*ROcp1_55)
    OPcp1_211 = OPcp1_29+ROcp1_55*qdd[11]+qd[11]*(-OMcp1_19*S5+OMcp1_39*ROcp1_45)
    OPcp1_311 = OPcp1_39+qdd[11]*S5+qd[11]*(OMcp1_19*ROcp1_55-OMcp1_29*ROcp1_45)
    ACcp1_111 = ACcp1_110+OMcp1_29*ORcp1_311-OMcp1_39*ORcp1_211+OPcp1_29*RLcp1_311-OPcp1_39*RLcp1_211
    ACcp1_211 = ACcp1_210-OMcp1_19*ORcp1_311+OMcp1_39*ORcp1_111-OPcp1_19*RLcp1_311+OPcp1_39*RLcp1_111
    ACcp1_311 = ACcp1_310+OMcp1_19*ORcp1_211-OMcp1_29*ORcp1_111+OPcp1_19*RLcp1_211-OPcp1_29*RLcp1_111
    OMcp1_112 = OMcp1_111+ROcp1_45*qd[12]
    OMcp1_212 = OMcp1_211+ROcp1_55*qd[12]
    OMcp1_312 = OMcp1_311+qd[12]*S5
    OPcp1_112 = OPcp1_111+ROcp1_45*qdd[12]+qd[12]*(OMcp1_211*S5-OMcp1_311*ROcp1_55)
    OPcp1_212 = OPcp1_211+ROcp1_55*qdd[12]+qd[12]*(-OMcp1_111*S5+OMcp1_311*ROcp1_45)
    OPcp1_312 = OPcp1_311+qdd[12]*S5+qd[12]*(OMcp1_111*ROcp1_55-OMcp1_211*ROcp1_45)
    OMcp1_113 = OMcp1_112+ROcp1_712*qd[13]
    OMcp1_213 = OMcp1_212+ROcp1_812*qd[13]
    OMcp1_313 = OMcp1_312+ROcp1_912*qd[13]
    OPcp1_113 = OPcp1_112+ROcp1_712*qdd[13]+qd[13]*(OMcp1_212*ROcp1_912-OMcp1_312*ROcp1_812)
    OPcp1_213 = OPcp1_212+ROcp1_812*qdd[13]+qd[13]*(-OMcp1_112*ROcp1_912+OMcp1_312*ROcp1_712)
    OPcp1_313 = OPcp1_312+ROcp1_912*qdd[13]+qd[13]*(OMcp1_112*ROcp1_812-OMcp1_212*ROcp1_712)
    RLcp1_114 = ROcp1_712*Dz143
    RLcp1_214 = ROcp1_812*Dz143
    RLcp1_314 = ROcp1_912*Dz143
    POcp1_114 = POcp1_111+RLcp1_114
    POcp1_214 = POcp1_211+RLcp1_214
    POcp1_314 = POcp1_311+RLcp1_314
    ORcp1_114 = OMcp1_213*RLcp1_314-OMcp1_313*RLcp1_214
    ORcp1_214 = -OMcp1_113*RLcp1_314+OMcp1_313*RLcp1_114
    ORcp1_314 = OMcp1_113*RLcp1_214-OMcp1_213*RLcp1_114
    VIcp1_114 = ORcp1_114+VIcp1_111+ROcp1_712*qd[14]
    VIcp1_214 = ORcp1_214+VIcp1_211+ROcp1_812*qd[14]
    VIcp1_314 = ORcp1_314+VIcp1_311+ROcp1_912*qd[14]
    ACcp1_114 = ACcp1_111+OMcp1_213*ORcp1_314-OMcp1_313*ORcp1_214+OPcp1_213*RLcp1_314-OPcp1_313*RLcp1_214+ROcp1_712*qdd[14]+(2.0)*qd[14]*(OMcp1_213*ROcp1_912-OMcp1_313*ROcp1_812)
    ACcp1_214 = ACcp1_211-OMcp1_113*ORcp1_314+OMcp1_313*ORcp1_114-OPcp1_113*RLcp1_314+OPcp1_313*RLcp1_114+ROcp1_812*qdd[14]+(2.0)*qd[14]*(-OMcp1_113*ROcp1_912+OMcp1_313*ROcp1_712)
    ACcp1_314 = ACcp1_311+OMcp1_113*ORcp1_214-OMcp1_213*ORcp1_114+OPcp1_113*RLcp1_214-OPcp1_213*RLcp1_114+ROcp1_912*qdd[14]+(2.0)*qd[14]*(OMcp1_113*ROcp1_812-OMcp1_213*ROcp1_712)
    RLcp1_115 = ROcp1_712*s.dpt[3,9]
    RLcp1_215 = ROcp1_812*s.dpt[3,9]
    RLcp1_315 = ROcp1_912*s.dpt[3,9]
    POcp1_115 = POcp1_114+RLcp1_115
    POcp1_215 = POcp1_214+RLcp1_215
    POcp1_315 = POcp1_314+RLcp1_315
    OMcp1_115 = OMcp1_113+ROcp1_413*qd[15]
    OMcp1_215 = OMcp1_213+ROcp1_513*qd[15]
    OMcp1_315 = OMcp1_313+ROcp1_613*qd[15]
    ORcp1_115 = OMcp1_213*RLcp1_315-OMcp1_313*RLcp1_215
    ORcp1_215 = -OMcp1_113*RLcp1_315+OMcp1_313*RLcp1_115
    ORcp1_315 = OMcp1_113*RLcp1_215-OMcp1_213*RLcp1_115
    VIcp1_115 = ORcp1_115+VIcp1_114
    VIcp1_215 = ORcp1_215+VIcp1_214
    VIcp1_315 = ORcp1_315+VIcp1_314
    OPcp1_115 = OPcp1_113+ROcp1_413*qdd[15]+qd[15]*(OMcp1_213*ROcp1_613-OMcp1_313*ROcp1_513)
    OPcp1_215 = OPcp1_213+ROcp1_513*qdd[15]+qd[15]*(-OMcp1_113*ROcp1_613+OMcp1_313*ROcp1_413)
    OPcp1_315 = OPcp1_313+ROcp1_613*qdd[15]+qd[15]*(OMcp1_113*ROcp1_513-OMcp1_213*ROcp1_413)
    ACcp1_115 = ACcp1_114+OMcp1_213*ORcp1_315-OMcp1_313*ORcp1_215+OPcp1_213*RLcp1_315-OPcp1_313*RLcp1_215
    ACcp1_215 = ACcp1_214-OMcp1_113*ORcp1_315+OMcp1_313*ORcp1_115-OPcp1_113*RLcp1_315+OPcp1_313*RLcp1_115
    ACcp1_315 = ACcp1_314+OMcp1_113*ORcp1_215-OMcp1_213*ORcp1_115+OPcp1_113*RLcp1_215-OPcp1_213*RLcp1_115
    sens.P[1] = POcp1_115
    sens.P[2] = POcp1_215
    sens.P[3] = POcp1_315
    sens.R[1,1] = ROcp1_115
    sens.R[1,2] = ROcp1_215
    sens.R[1,3] = ROcp1_315
    sens.R[2,1] = ROcp1_413
    sens.R[2,2] = ROcp1_513
    sens.R[2,3] = ROcp1_613
    sens.R[3,1] = ROcp1_715
    sens.R[3,2] = ROcp1_815
    sens.R[3,3] = ROcp1_915
    sens.V[1] = VIcp1_115
    sens.V[2] = VIcp1_215
    sens.V[3] = VIcp1_315
    sens.OM[1] = OMcp1_115
    sens.OM[2] = OMcp1_215
    sens.OM[3] = OMcp1_315
    sens.A[1] = ACcp1_115
    sens.A[2] = ACcp1_215
    sens.A[3] = ACcp1_315
    sens.OMP[1] = OPcp1_115
    sens.OMP[2] = OPcp1_215
    sens.OMP[3] = OPcp1_315

  if (isens == 2): 

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
    ROcp2_19 = ROcp2_18*C9-ROcp2_78*S9
    ROcp2_29 = ROcp2_28*C9-ROcp2_88*S9
    ROcp2_39 = ROcp2_38*C9-ROcp2_98*S9
    ROcp2_79 = ROcp2_18*S9+ROcp2_78*C9
    ROcp2_89 = ROcp2_28*S9+ROcp2_88*C9
    ROcp2_99 = ROcp2_38*S9+ROcp2_98*C9
    ROcp2_111 = ROcp2_19*C11-ROcp2_79*S11
    ROcp2_211 = ROcp2_29*C11-ROcp2_89*S11
    ROcp2_311 = ROcp2_39*C11-ROcp2_99*S11
    ROcp2_711 = ROcp2_19*S11+ROcp2_79*C11
    ROcp2_811 = ROcp2_29*S11+ROcp2_89*C11
    ROcp2_911 = ROcp2_39*S11+ROcp2_99*C11
    ROcp2_117 = ROcp2_111*C17-ROcp2_711*S17
    ROcp2_217 = ROcp2_211*C17-ROcp2_811*S17
    ROcp2_317 = ROcp2_311*C17-ROcp2_911*S17
    ROcp2_717 = ROcp2_111*S17+ROcp2_711*C17
    ROcp2_817 = ROcp2_211*S17+ROcp2_811*C17
    ROcp2_917 = ROcp2_311*S17+ROcp2_911*C17
    ROcp2_118 = ROcp2_117*C18-ROcp2_717*S18
    ROcp2_218 = ROcp2_217*C18-ROcp2_817*S18
    ROcp2_318 = ROcp2_317*C18-ROcp2_917*S18
    ROcp2_718 = ROcp2_117*S18+ROcp2_717*C18
    ROcp2_818 = ROcp2_217*S18+ROcp2_817*C18
    ROcp2_918 = ROcp2_317*S18+ROcp2_917*C18
    ROcp2_119 = ROcp2_118*C19-ROcp2_718*S19
    ROcp2_219 = ROcp2_218*C19-ROcp2_818*S19
    ROcp2_319 = ROcp2_318*C19-ROcp2_918*S19
    ROcp2_719 = ROcp2_118*S19+ROcp2_718*C19
    ROcp2_819 = ROcp2_218*S19+ROcp2_818*C19
    ROcp2_919 = ROcp2_318*S19+ROcp2_918*C19
    OMcp2_15 = qd[5]*C4
    OMcp2_25 = qd[5]*S4
    OPcp2_15 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp2_25 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp2_16 = OMcp2_15+ROcp2_45*qd[6]
    OMcp2_26 = OMcp2_25+ROcp2_55*qd[6]
    OMcp2_36 = qd[4]+qd[6]*S5
    OPcp2_16 = OPcp2_15+ROcp2_45*qdd[6]+qd[6]*(OMcp2_25*S5-ROcp2_55*qd[4])
    OPcp2_26 = OPcp2_25+ROcp2_55*qdd[6]+qd[6]*(-OMcp2_15*S5+ROcp2_45*qd[4])
    OPcp2_36 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp2_15*ROcp2_55-OMcp2_25*ROcp2_45)
    RLcp2_17 = ROcp2_16*s.dpt[1,1]
    RLcp2_27 = ROcp2_26*s.dpt[1,1]
    RLcp2_37 = ROcp2_36*s.dpt[1,1]
    POcp2_17 = RLcp2_17+q[1]
    POcp2_27 = RLcp2_27+q[2]
    POcp2_37 = RLcp2_37+q[3]
    OMcp2_17 = OMcp2_16+ROcp2_45*qd[7]
    OMcp2_27 = OMcp2_26+ROcp2_55*qd[7]
    OMcp2_37 = OMcp2_36+qd[7]*S5
    ORcp2_17 = OMcp2_26*RLcp2_37-OMcp2_36*RLcp2_27
    ORcp2_27 = -OMcp2_16*RLcp2_37+OMcp2_36*RLcp2_17
    ORcp2_37 = OMcp2_16*RLcp2_27-OMcp2_26*RLcp2_17
    VIcp2_17 = ORcp2_17+qd[1]
    VIcp2_27 = ORcp2_27+qd[2]
    VIcp2_37 = ORcp2_37+qd[3]
    OPcp2_17 = OPcp2_16+ROcp2_45*qdd[7]+qd[7]*(OMcp2_26*S5-OMcp2_36*ROcp2_55)
    OPcp2_27 = OPcp2_26+ROcp2_55*qdd[7]+qd[7]*(-OMcp2_16*S5+OMcp2_36*ROcp2_45)
    OPcp2_37 = OPcp2_36+qdd[7]*S5+qd[7]*(OMcp2_16*ROcp2_55-OMcp2_26*ROcp2_45)
    ACcp2_17 = qdd[1]+OMcp2_26*ORcp2_37-OMcp2_36*ORcp2_27+OPcp2_26*RLcp2_37-OPcp2_36*RLcp2_27
    ACcp2_27 = qdd[2]-OMcp2_16*ORcp2_37+OMcp2_36*ORcp2_17-OPcp2_16*RLcp2_37+OPcp2_36*RLcp2_17
    ACcp2_37 = qdd[3]+OMcp2_16*ORcp2_27-OMcp2_26*ORcp2_17+OPcp2_16*RLcp2_27-OPcp2_26*RLcp2_17
    RLcp2_18 = ROcp2_17*s.dpt[1,2]
    RLcp2_28 = ROcp2_27*s.dpt[1,2]
    RLcp2_38 = ROcp2_37*s.dpt[1,2]
    POcp2_18 = POcp2_17+RLcp2_18
    POcp2_28 = POcp2_27+RLcp2_28
    POcp2_38 = POcp2_37+RLcp2_38
    OMcp2_18 = OMcp2_17+ROcp2_45*qd[8]
    OMcp2_28 = OMcp2_27+ROcp2_55*qd[8]
    OMcp2_38 = OMcp2_37+qd[8]*S5
    ORcp2_18 = OMcp2_27*RLcp2_38-OMcp2_37*RLcp2_28
    ORcp2_28 = -OMcp2_17*RLcp2_38+OMcp2_37*RLcp2_18
    ORcp2_38 = OMcp2_17*RLcp2_28-OMcp2_27*RLcp2_18
    VIcp2_18 = ORcp2_18+VIcp2_17
    VIcp2_28 = ORcp2_28+VIcp2_27
    VIcp2_38 = ORcp2_38+VIcp2_37
    OPcp2_18 = OPcp2_17+ROcp2_45*qdd[8]+qd[8]*(OMcp2_27*S5-OMcp2_37*ROcp2_55)
    OPcp2_28 = OPcp2_27+ROcp2_55*qdd[8]+qd[8]*(-OMcp2_17*S5+OMcp2_37*ROcp2_45)
    OPcp2_38 = OPcp2_37+qdd[8]*S5+qd[8]*(OMcp2_17*ROcp2_55-OMcp2_27*ROcp2_45)
    ACcp2_18 = ACcp2_17+OMcp2_27*ORcp2_38-OMcp2_37*ORcp2_28+OPcp2_27*RLcp2_38-OPcp2_37*RLcp2_28
    ACcp2_28 = ACcp2_27-OMcp2_17*ORcp2_38+OMcp2_37*ORcp2_18-OPcp2_17*RLcp2_38+OPcp2_37*RLcp2_18
    ACcp2_38 = ACcp2_37+OMcp2_17*ORcp2_28-OMcp2_27*ORcp2_18+OPcp2_17*RLcp2_28-OPcp2_27*RLcp2_18
    RLcp2_19 = ROcp2_18*s.dpt[1,3]
    RLcp2_29 = ROcp2_28*s.dpt[1,3]
    RLcp2_39 = ROcp2_38*s.dpt[1,3]
    POcp2_19 = POcp2_18+RLcp2_19
    POcp2_29 = POcp2_28+RLcp2_29
    POcp2_39 = POcp2_38+RLcp2_39
    OMcp2_19 = OMcp2_18+ROcp2_45*qd[9]
    OMcp2_29 = OMcp2_28+ROcp2_55*qd[9]
    OMcp2_39 = OMcp2_38+qd[9]*S5
    ORcp2_19 = OMcp2_28*RLcp2_39-OMcp2_38*RLcp2_29
    ORcp2_29 = -OMcp2_18*RLcp2_39+OMcp2_38*RLcp2_19
    ORcp2_39 = OMcp2_18*RLcp2_29-OMcp2_28*RLcp2_19
    VIcp2_19 = ORcp2_19+VIcp2_18
    VIcp2_29 = ORcp2_29+VIcp2_28
    VIcp2_39 = ORcp2_39+VIcp2_38
    OPcp2_19 = OPcp2_18+ROcp2_45*qdd[9]+qd[9]*(OMcp2_28*S5-OMcp2_38*ROcp2_55)
    OPcp2_29 = OPcp2_28+ROcp2_55*qdd[9]+qd[9]*(-OMcp2_18*S5+OMcp2_38*ROcp2_45)
    OPcp2_39 = OPcp2_38+qdd[9]*S5+qd[9]*(OMcp2_18*ROcp2_55-OMcp2_28*ROcp2_45)
    ACcp2_19 = ACcp2_18+OMcp2_28*ORcp2_39-OMcp2_38*ORcp2_29+OPcp2_28*RLcp2_39-OPcp2_38*RLcp2_29
    ACcp2_29 = ACcp2_28-OMcp2_18*ORcp2_39+OMcp2_38*ORcp2_19-OPcp2_18*RLcp2_39+OPcp2_38*RLcp2_19
    ACcp2_39 = ACcp2_38+OMcp2_18*ORcp2_29-OMcp2_28*ORcp2_19+OPcp2_18*RLcp2_29-OPcp2_28*RLcp2_19
    RLcp2_110 = ROcp2_19*Dz101
    RLcp2_210 = ROcp2_29*Dz101
    RLcp2_310 = ROcp2_39*Dz101
    POcp2_110 = POcp2_19+RLcp2_110
    POcp2_210 = POcp2_29+RLcp2_210
    POcp2_310 = POcp2_39+RLcp2_310
    ORcp2_110 = OMcp2_29*RLcp2_310-OMcp2_39*RLcp2_210
    ORcp2_210 = -OMcp2_19*RLcp2_310+OMcp2_39*RLcp2_110
    ORcp2_310 = OMcp2_19*RLcp2_210-OMcp2_29*RLcp2_110
    VIcp2_110 = ORcp2_110+VIcp2_19+ROcp2_19*qd[10]
    VIcp2_210 = ORcp2_210+VIcp2_29+ROcp2_29*qd[10]
    VIcp2_310 = ORcp2_310+VIcp2_39+ROcp2_39*qd[10]
    ACcp2_110 = ACcp2_19+OMcp2_29*ORcp2_310-OMcp2_39*ORcp2_210+OPcp2_29*RLcp2_310-OPcp2_39*RLcp2_210+ROcp2_19*qdd[10]+(2.0)*qd[10]*(OMcp2_29*ROcp2_39-OMcp2_39*ROcp2_29)
    ACcp2_210 = ACcp2_29-OMcp2_19*ORcp2_310+OMcp2_39*ORcp2_110-OPcp2_19*RLcp2_310+OPcp2_39*RLcp2_110+ROcp2_29*qdd[10]+(2.0)*qd[10]*(-OMcp2_19*ROcp2_39+OMcp2_39*ROcp2_19)
    ACcp2_310 = ACcp2_39+OMcp2_19*ORcp2_210-OMcp2_29*ORcp2_110+OPcp2_19*RLcp2_210-OPcp2_29*RLcp2_110+ROcp2_39*qdd[10]+(2.0)*qd[10]*(OMcp2_19*ROcp2_29-OMcp2_29*ROcp2_19)
    RLcp2_111 = ROcp2_19*s.dpt[1,5]
    RLcp2_211 = ROcp2_29*s.dpt[1,5]
    RLcp2_311 = ROcp2_39*s.dpt[1,5]
    POcp2_111 = POcp2_110+RLcp2_111
    POcp2_211 = POcp2_210+RLcp2_211
    POcp2_311 = POcp2_310+RLcp2_311
    OMcp2_111 = OMcp2_19+ROcp2_45*qd[11]
    OMcp2_211 = OMcp2_29+ROcp2_55*qd[11]
    OMcp2_311 = OMcp2_39+qd[11]*S5
    ORcp2_111 = OMcp2_29*RLcp2_311-OMcp2_39*RLcp2_211
    ORcp2_211 = -OMcp2_19*RLcp2_311+OMcp2_39*RLcp2_111
    ORcp2_311 = OMcp2_19*RLcp2_211-OMcp2_29*RLcp2_111
    VIcp2_111 = ORcp2_111+VIcp2_110
    VIcp2_211 = ORcp2_211+VIcp2_210
    VIcp2_311 = ORcp2_311+VIcp2_310
    OPcp2_111 = OPcp2_19+ROcp2_45*qdd[11]+qd[11]*(OMcp2_29*S5-OMcp2_39*ROcp2_55)
    OPcp2_211 = OPcp2_29+ROcp2_55*qdd[11]+qd[11]*(-OMcp2_19*S5+OMcp2_39*ROcp2_45)
    OPcp2_311 = OPcp2_39+qdd[11]*S5+qd[11]*(OMcp2_19*ROcp2_55-OMcp2_29*ROcp2_45)
    ACcp2_111 = ACcp2_110+OMcp2_29*ORcp2_311-OMcp2_39*ORcp2_211+OPcp2_29*RLcp2_311-OPcp2_39*RLcp2_211
    ACcp2_211 = ACcp2_210-OMcp2_19*ORcp2_311+OMcp2_39*ORcp2_111-OPcp2_19*RLcp2_311+OPcp2_39*RLcp2_111
    ACcp2_311 = ACcp2_310+OMcp2_19*ORcp2_211-OMcp2_29*ORcp2_111+OPcp2_19*RLcp2_211-OPcp2_29*RLcp2_111
    RLcp2_112 = ROcp2_45*s.dpt[2,6]+ROcp2_711*s.dpt[3,6]
    RLcp2_212 = ROcp2_55*s.dpt[2,6]+ROcp2_811*s.dpt[3,6]
    RLcp2_312 = ROcp2_911*s.dpt[3,6]+s.dpt[2,6]*S5
    POcp2_112 = POcp2_111+RLcp2_112
    POcp2_212 = POcp2_211+RLcp2_212
    POcp2_312 = POcp2_311+RLcp2_312
    OMcp2_112 = OMcp2_111+ROcp2_45*qd[17]
    OMcp2_212 = OMcp2_211+ROcp2_55*qd[17]
    OMcp2_312 = OMcp2_311+qd[17]*S5
    ORcp2_112 = OMcp2_211*RLcp2_312-OMcp2_311*RLcp2_212
    ORcp2_212 = -OMcp2_111*RLcp2_312+OMcp2_311*RLcp2_112
    ORcp2_312 = OMcp2_111*RLcp2_212-OMcp2_211*RLcp2_112
    VIcp2_112 = ORcp2_112+VIcp2_111
    VIcp2_212 = ORcp2_212+VIcp2_211
    VIcp2_312 = ORcp2_312+VIcp2_311
    OPcp2_112 = OPcp2_111+ROcp2_45*qdd[17]+qd[17]*(OMcp2_211*S5-OMcp2_311*ROcp2_55)
    OPcp2_212 = OPcp2_211+ROcp2_55*qdd[17]+qd[17]*(-OMcp2_111*S5+OMcp2_311*ROcp2_45)
    OPcp2_312 = OPcp2_311+qdd[17]*S5+qd[17]*(OMcp2_111*ROcp2_55-OMcp2_211*ROcp2_45)
    ACcp2_112 = ACcp2_111+OMcp2_211*ORcp2_312-OMcp2_311*ORcp2_212+OPcp2_211*RLcp2_312-OPcp2_311*RLcp2_212
    ACcp2_212 = ACcp2_211-OMcp2_111*ORcp2_312+OMcp2_311*ORcp2_112-OPcp2_111*RLcp2_312+OPcp2_311*RLcp2_112
    ACcp2_312 = ACcp2_311+OMcp2_111*ORcp2_212-OMcp2_211*ORcp2_112+OPcp2_111*RLcp2_212-OPcp2_211*RLcp2_112
    RLcp2_113 = ROcp2_117*s.dpt[1,13]+ROcp2_717*s.dpt[3,13]
    RLcp2_213 = ROcp2_217*s.dpt[1,13]+ROcp2_817*s.dpt[3,13]
    RLcp2_313 = ROcp2_317*s.dpt[1,13]+ROcp2_917*s.dpt[3,13]
    POcp2_113 = POcp2_112+RLcp2_113
    POcp2_213 = POcp2_212+RLcp2_213
    POcp2_313 = POcp2_312+RLcp2_313
    OMcp2_113 = OMcp2_112+ROcp2_45*qd[18]
    OMcp2_213 = OMcp2_212+ROcp2_55*qd[18]
    OMcp2_313 = OMcp2_312+qd[18]*S5
    ORcp2_113 = OMcp2_212*RLcp2_313-OMcp2_312*RLcp2_213
    ORcp2_213 = -OMcp2_112*RLcp2_313+OMcp2_312*RLcp2_113
    ORcp2_313 = OMcp2_112*RLcp2_213-OMcp2_212*RLcp2_113
    VIcp2_113 = ORcp2_113+VIcp2_112
    VIcp2_213 = ORcp2_213+VIcp2_212
    VIcp2_313 = ORcp2_313+VIcp2_312
    OPcp2_113 = OPcp2_112+ROcp2_45*qdd[18]+qd[18]*(OMcp2_212*S5-OMcp2_312*ROcp2_55)
    OPcp2_213 = OPcp2_212+ROcp2_55*qdd[18]+qd[18]*(-OMcp2_112*S5+OMcp2_312*ROcp2_45)
    OPcp2_313 = OPcp2_312+qdd[18]*S5+qd[18]*(OMcp2_112*ROcp2_55-OMcp2_212*ROcp2_45)
    ACcp2_113 = ACcp2_112+OMcp2_212*ORcp2_313-OMcp2_312*ORcp2_213+OPcp2_212*RLcp2_313-OPcp2_312*RLcp2_213
    ACcp2_213 = ACcp2_212-OMcp2_112*ORcp2_313+OMcp2_312*ORcp2_113-OPcp2_112*RLcp2_313+OPcp2_312*RLcp2_113
    ACcp2_313 = ACcp2_312+OMcp2_112*ORcp2_213-OMcp2_212*ORcp2_113+OPcp2_112*RLcp2_213-OPcp2_212*RLcp2_113
    RLcp2_114 = ROcp2_118*s.dpt[1,15]
    RLcp2_214 = ROcp2_218*s.dpt[1,15]
    RLcp2_314 = ROcp2_318*s.dpt[1,15]
    POcp2_114 = POcp2_113+RLcp2_114
    POcp2_214 = POcp2_213+RLcp2_214
    POcp2_314 = POcp2_313+RLcp2_314
    OMcp2_114 = OMcp2_113+ROcp2_45*qd[19]
    OMcp2_214 = OMcp2_213+ROcp2_55*qd[19]
    OMcp2_314 = OMcp2_313+qd[19]*S5
    ORcp2_114 = OMcp2_213*RLcp2_314-OMcp2_313*RLcp2_214
    ORcp2_214 = -OMcp2_113*RLcp2_314+OMcp2_313*RLcp2_114
    ORcp2_314 = OMcp2_113*RLcp2_214-OMcp2_213*RLcp2_114
    VIcp2_114 = ORcp2_114+VIcp2_113
    VIcp2_214 = ORcp2_214+VIcp2_213
    VIcp2_314 = ORcp2_314+VIcp2_313
    OPcp2_114 = OPcp2_113+ROcp2_45*qdd[19]+qd[19]*(OMcp2_213*S5-OMcp2_313*ROcp2_55)
    OPcp2_214 = OPcp2_213+ROcp2_55*qdd[19]+qd[19]*(-OMcp2_113*S5+OMcp2_313*ROcp2_45)
    OPcp2_314 = OPcp2_313+qdd[19]*S5+qd[19]*(OMcp2_113*ROcp2_55-OMcp2_213*ROcp2_45)
    ACcp2_114 = ACcp2_113+OMcp2_213*ORcp2_314-OMcp2_313*ORcp2_214+OPcp2_213*RLcp2_314-OPcp2_313*RLcp2_214
    ACcp2_214 = ACcp2_213-OMcp2_113*ORcp2_314+OMcp2_313*ORcp2_114-OPcp2_113*RLcp2_314+OPcp2_313*RLcp2_114
    ACcp2_314 = ACcp2_313+OMcp2_113*ORcp2_214-OMcp2_213*ORcp2_114+OPcp2_113*RLcp2_214-OPcp2_213*RLcp2_114
    sens.P[1] = POcp2_114
    sens.P[2] = POcp2_214
    sens.P[3] = POcp2_314
    sens.R[1,1] = ROcp2_119
    sens.R[1,2] = ROcp2_219
    sens.R[1,3] = ROcp2_319
    sens.R[2,1] = ROcp2_45
    sens.R[2,2] = ROcp2_55
    sens.R[2,3] = S5
    sens.R[3,1] = ROcp2_719
    sens.R[3,2] = ROcp2_819
    sens.R[3,3] = ROcp2_919
    sens.V[1] = VIcp2_114
    sens.V[2] = VIcp2_214
    sens.V[3] = VIcp2_314
    sens.OM[1] = OMcp2_114
    sens.OM[2] = OMcp2_214
    sens.OM[3] = OMcp2_314
    sens.A[1] = ACcp2_114
    sens.A[2] = ACcp2_214
    sens.A[3] = ACcp2_314
    sens.OMP[1] = OPcp2_114
    sens.OMP[2] = OPcp2_214
    sens.OMP[3] = OPcp2_314

  if (isens == 3): 

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
    ROcp3_19 = ROcp3_18*C9-ROcp3_78*S9
    ROcp3_29 = ROcp3_28*C9-ROcp3_88*S9
    ROcp3_39 = ROcp3_38*C9-ROcp3_98*S9
    ROcp3_79 = ROcp3_18*S9+ROcp3_78*C9
    ROcp3_89 = ROcp3_28*S9+ROcp3_88*C9
    ROcp3_99 = ROcp3_38*S9+ROcp3_98*C9
    ROcp3_111 = ROcp3_19*C11-ROcp3_79*S11
    ROcp3_211 = ROcp3_29*C11-ROcp3_89*S11
    ROcp3_311 = ROcp3_39*C11-ROcp3_99*S11
    ROcp3_711 = ROcp3_19*S11+ROcp3_79*C11
    ROcp3_811 = ROcp3_29*S11+ROcp3_89*C11
    ROcp3_911 = ROcp3_39*S11+ROcp3_99*C11
    ROcp3_120 = ROcp3_111*C20-ROcp3_711*S20
    ROcp3_220 = ROcp3_211*C20-ROcp3_811*S20
    ROcp3_320 = ROcp3_311*C20-ROcp3_911*S20
    ROcp3_720 = ROcp3_111*S20+ROcp3_711*C20
    ROcp3_820 = ROcp3_211*S20+ROcp3_811*C20
    ROcp3_920 = ROcp3_311*S20+ROcp3_911*C20
    ROcp3_121 = ROcp3_120*C21-ROcp3_720*S21
    ROcp3_221 = ROcp3_220*C21-ROcp3_820*S21
    ROcp3_321 = ROcp3_320*C21-ROcp3_920*S21
    ROcp3_721 = ROcp3_120*S21+ROcp3_720*C21
    ROcp3_821 = ROcp3_220*S21+ROcp3_820*C21
    ROcp3_921 = ROcp3_320*S21+ROcp3_920*C21
    ROcp3_122 = ROcp3_121*C22-ROcp3_721*S22
    ROcp3_222 = ROcp3_221*C22-ROcp3_821*S22
    ROcp3_322 = ROcp3_321*C22-ROcp3_921*S22
    ROcp3_722 = ROcp3_121*S22+ROcp3_721*C22
    ROcp3_822 = ROcp3_221*S22+ROcp3_821*C22
    ROcp3_922 = ROcp3_321*S22+ROcp3_921*C22
    OMcp3_15 = qd[5]*C4
    OMcp3_25 = qd[5]*S4
    OPcp3_15 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp3_25 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp3_16 = OMcp3_15+ROcp3_45*qd[6]
    OMcp3_26 = OMcp3_25+ROcp3_55*qd[6]
    OMcp3_36 = qd[4]+qd[6]*S5
    OPcp3_16 = OPcp3_15+ROcp3_45*qdd[6]+qd[6]*(OMcp3_25*S5-ROcp3_55*qd[4])
    OPcp3_26 = OPcp3_25+ROcp3_55*qdd[6]+qd[6]*(-OMcp3_15*S5+ROcp3_45*qd[4])
    OPcp3_36 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp3_15*ROcp3_55-OMcp3_25*ROcp3_45)
    RLcp3_17 = ROcp3_16*s.dpt[1,1]
    RLcp3_27 = ROcp3_26*s.dpt[1,1]
    RLcp3_37 = ROcp3_36*s.dpt[1,1]
    POcp3_17 = RLcp3_17+q[1]
    POcp3_27 = RLcp3_27+q[2]
    POcp3_37 = RLcp3_37+q[3]
    OMcp3_17 = OMcp3_16+ROcp3_45*qd[7]
    OMcp3_27 = OMcp3_26+ROcp3_55*qd[7]
    OMcp3_37 = OMcp3_36+qd[7]*S5
    ORcp3_17 = OMcp3_26*RLcp3_37-OMcp3_36*RLcp3_27
    ORcp3_27 = -OMcp3_16*RLcp3_37+OMcp3_36*RLcp3_17
    ORcp3_37 = OMcp3_16*RLcp3_27-OMcp3_26*RLcp3_17
    VIcp3_17 = ORcp3_17+qd[1]
    VIcp3_27 = ORcp3_27+qd[2]
    VIcp3_37 = ORcp3_37+qd[3]
    OPcp3_17 = OPcp3_16+ROcp3_45*qdd[7]+qd[7]*(OMcp3_26*S5-OMcp3_36*ROcp3_55)
    OPcp3_27 = OPcp3_26+ROcp3_55*qdd[7]+qd[7]*(-OMcp3_16*S5+OMcp3_36*ROcp3_45)
    OPcp3_37 = OPcp3_36+qdd[7]*S5+qd[7]*(OMcp3_16*ROcp3_55-OMcp3_26*ROcp3_45)
    ACcp3_17 = qdd[1]+OMcp3_26*ORcp3_37-OMcp3_36*ORcp3_27+OPcp3_26*RLcp3_37-OPcp3_36*RLcp3_27
    ACcp3_27 = qdd[2]-OMcp3_16*ORcp3_37+OMcp3_36*ORcp3_17-OPcp3_16*RLcp3_37+OPcp3_36*RLcp3_17
    ACcp3_37 = qdd[3]+OMcp3_16*ORcp3_27-OMcp3_26*ORcp3_17+OPcp3_16*RLcp3_27-OPcp3_26*RLcp3_17
    RLcp3_18 = ROcp3_17*s.dpt[1,2]
    RLcp3_28 = ROcp3_27*s.dpt[1,2]
    RLcp3_38 = ROcp3_37*s.dpt[1,2]
    POcp3_18 = POcp3_17+RLcp3_18
    POcp3_28 = POcp3_27+RLcp3_28
    POcp3_38 = POcp3_37+RLcp3_38
    OMcp3_18 = OMcp3_17+ROcp3_45*qd[8]
    OMcp3_28 = OMcp3_27+ROcp3_55*qd[8]
    OMcp3_38 = OMcp3_37+qd[8]*S5
    ORcp3_18 = OMcp3_27*RLcp3_38-OMcp3_37*RLcp3_28
    ORcp3_28 = -OMcp3_17*RLcp3_38+OMcp3_37*RLcp3_18
    ORcp3_38 = OMcp3_17*RLcp3_28-OMcp3_27*RLcp3_18
    VIcp3_18 = ORcp3_18+VIcp3_17
    VIcp3_28 = ORcp3_28+VIcp3_27
    VIcp3_38 = ORcp3_38+VIcp3_37
    OPcp3_18 = OPcp3_17+ROcp3_45*qdd[8]+qd[8]*(OMcp3_27*S5-OMcp3_37*ROcp3_55)
    OPcp3_28 = OPcp3_27+ROcp3_55*qdd[8]+qd[8]*(-OMcp3_17*S5+OMcp3_37*ROcp3_45)
    OPcp3_38 = OPcp3_37+qdd[8]*S5+qd[8]*(OMcp3_17*ROcp3_55-OMcp3_27*ROcp3_45)
    ACcp3_18 = ACcp3_17+OMcp3_27*ORcp3_38-OMcp3_37*ORcp3_28+OPcp3_27*RLcp3_38-OPcp3_37*RLcp3_28
    ACcp3_28 = ACcp3_27-OMcp3_17*ORcp3_38+OMcp3_37*ORcp3_18-OPcp3_17*RLcp3_38+OPcp3_37*RLcp3_18
    ACcp3_38 = ACcp3_37+OMcp3_17*ORcp3_28-OMcp3_27*ORcp3_18+OPcp3_17*RLcp3_28-OPcp3_27*RLcp3_18
    RLcp3_19 = ROcp3_18*s.dpt[1,3]
    RLcp3_29 = ROcp3_28*s.dpt[1,3]
    RLcp3_39 = ROcp3_38*s.dpt[1,3]
    POcp3_19 = POcp3_18+RLcp3_19
    POcp3_29 = POcp3_28+RLcp3_29
    POcp3_39 = POcp3_38+RLcp3_39
    OMcp3_19 = OMcp3_18+ROcp3_45*qd[9]
    OMcp3_29 = OMcp3_28+ROcp3_55*qd[9]
    OMcp3_39 = OMcp3_38+qd[9]*S5
    ORcp3_19 = OMcp3_28*RLcp3_39-OMcp3_38*RLcp3_29
    ORcp3_29 = -OMcp3_18*RLcp3_39+OMcp3_38*RLcp3_19
    ORcp3_39 = OMcp3_18*RLcp3_29-OMcp3_28*RLcp3_19
    VIcp3_19 = ORcp3_19+VIcp3_18
    VIcp3_29 = ORcp3_29+VIcp3_28
    VIcp3_39 = ORcp3_39+VIcp3_38
    OPcp3_19 = OPcp3_18+ROcp3_45*qdd[9]+qd[9]*(OMcp3_28*S5-OMcp3_38*ROcp3_55)
    OPcp3_29 = OPcp3_28+ROcp3_55*qdd[9]+qd[9]*(-OMcp3_18*S5+OMcp3_38*ROcp3_45)
    OPcp3_39 = OPcp3_38+qdd[9]*S5+qd[9]*(OMcp3_18*ROcp3_55-OMcp3_28*ROcp3_45)
    ACcp3_19 = ACcp3_18+OMcp3_28*ORcp3_39-OMcp3_38*ORcp3_29+OPcp3_28*RLcp3_39-OPcp3_38*RLcp3_29
    ACcp3_29 = ACcp3_28-OMcp3_18*ORcp3_39+OMcp3_38*ORcp3_19-OPcp3_18*RLcp3_39+OPcp3_38*RLcp3_19
    ACcp3_39 = ACcp3_38+OMcp3_18*ORcp3_29-OMcp3_28*ORcp3_19+OPcp3_18*RLcp3_29-OPcp3_28*RLcp3_19
    RLcp3_110 = ROcp3_19*Dz101
    RLcp3_210 = ROcp3_29*Dz101
    RLcp3_310 = ROcp3_39*Dz101
    POcp3_110 = POcp3_19+RLcp3_110
    POcp3_210 = POcp3_29+RLcp3_210
    POcp3_310 = POcp3_39+RLcp3_310
    ORcp3_110 = OMcp3_29*RLcp3_310-OMcp3_39*RLcp3_210
    ORcp3_210 = -OMcp3_19*RLcp3_310+OMcp3_39*RLcp3_110
    ORcp3_310 = OMcp3_19*RLcp3_210-OMcp3_29*RLcp3_110
    VIcp3_110 = ORcp3_110+VIcp3_19+ROcp3_19*qd[10]
    VIcp3_210 = ORcp3_210+VIcp3_29+ROcp3_29*qd[10]
    VIcp3_310 = ORcp3_310+VIcp3_39+ROcp3_39*qd[10]
    ACcp3_110 = ACcp3_19+OMcp3_29*ORcp3_310-OMcp3_39*ORcp3_210+OPcp3_29*RLcp3_310-OPcp3_39*RLcp3_210+ROcp3_19*qdd[10]+(2.0)*qd[10]*(OMcp3_29*ROcp3_39-OMcp3_39*ROcp3_29)
    ACcp3_210 = ACcp3_29-OMcp3_19*ORcp3_310+OMcp3_39*ORcp3_110-OPcp3_19*RLcp3_310+OPcp3_39*RLcp3_110+ROcp3_29*qdd[10]+(2.0)*qd[10]*(-OMcp3_19*ROcp3_39+OMcp3_39*ROcp3_19)
    ACcp3_310 = ACcp3_39+OMcp3_19*ORcp3_210-OMcp3_29*ORcp3_110+OPcp3_19*RLcp3_210-OPcp3_29*RLcp3_110+ROcp3_39*qdd[10]+(2.0)*qd[10]*(OMcp3_19*ROcp3_29-OMcp3_29*ROcp3_19)
    RLcp3_111 = ROcp3_19*s.dpt[1,5]
    RLcp3_211 = ROcp3_29*s.dpt[1,5]
    RLcp3_311 = ROcp3_39*s.dpt[1,5]
    POcp3_111 = POcp3_110+RLcp3_111
    POcp3_211 = POcp3_210+RLcp3_211
    POcp3_311 = POcp3_310+RLcp3_311
    OMcp3_111 = OMcp3_19+ROcp3_45*qd[11]
    OMcp3_211 = OMcp3_29+ROcp3_55*qd[11]
    OMcp3_311 = OMcp3_39+qd[11]*S5
    ORcp3_111 = OMcp3_29*RLcp3_311-OMcp3_39*RLcp3_211
    ORcp3_211 = -OMcp3_19*RLcp3_311+OMcp3_39*RLcp3_111
    ORcp3_311 = OMcp3_19*RLcp3_211-OMcp3_29*RLcp3_111
    VIcp3_111 = ORcp3_111+VIcp3_110
    VIcp3_211 = ORcp3_211+VIcp3_210
    VIcp3_311 = ORcp3_311+VIcp3_310
    OPcp3_111 = OPcp3_19+ROcp3_45*qdd[11]+qd[11]*(OMcp3_29*S5-OMcp3_39*ROcp3_55)
    OPcp3_211 = OPcp3_29+ROcp3_55*qdd[11]+qd[11]*(-OMcp3_19*S5+OMcp3_39*ROcp3_45)
    OPcp3_311 = OPcp3_39+qdd[11]*S5+qd[11]*(OMcp3_19*ROcp3_55-OMcp3_29*ROcp3_45)
    ACcp3_111 = ACcp3_110+OMcp3_29*ORcp3_311-OMcp3_39*ORcp3_211+OPcp3_29*RLcp3_311-OPcp3_39*RLcp3_211
    ACcp3_211 = ACcp3_210-OMcp3_19*ORcp3_311+OMcp3_39*ORcp3_111-OPcp3_19*RLcp3_311+OPcp3_39*RLcp3_111
    ACcp3_311 = ACcp3_310+OMcp3_19*ORcp3_211-OMcp3_29*ORcp3_111+OPcp3_19*RLcp3_211-OPcp3_29*RLcp3_111
    RLcp3_112 = ROcp3_45*s.dpt[2,7]+ROcp3_711*s.dpt[3,7]
    RLcp3_212 = ROcp3_55*s.dpt[2,7]+ROcp3_811*s.dpt[3,7]
    RLcp3_312 = ROcp3_911*s.dpt[3,7]+s.dpt[2,7]*S5
    POcp3_112 = POcp3_111+RLcp3_112
    POcp3_212 = POcp3_211+RLcp3_212
    POcp3_312 = POcp3_311+RLcp3_312
    OMcp3_112 = OMcp3_111+ROcp3_45*qd[20]
    OMcp3_212 = OMcp3_211+ROcp3_55*qd[20]
    OMcp3_312 = OMcp3_311+qd[20]*S5
    ORcp3_112 = OMcp3_211*RLcp3_312-OMcp3_311*RLcp3_212
    ORcp3_212 = -OMcp3_111*RLcp3_312+OMcp3_311*RLcp3_112
    ORcp3_312 = OMcp3_111*RLcp3_212-OMcp3_211*RLcp3_112
    VIcp3_112 = ORcp3_112+VIcp3_111
    VIcp3_212 = ORcp3_212+VIcp3_211
    VIcp3_312 = ORcp3_312+VIcp3_311
    OPcp3_112 = OPcp3_111+ROcp3_45*qdd[20]+qd[20]*(OMcp3_211*S5-OMcp3_311*ROcp3_55)
    OPcp3_212 = OPcp3_211+ROcp3_55*qdd[20]+qd[20]*(-OMcp3_111*S5+OMcp3_311*ROcp3_45)
    OPcp3_312 = OPcp3_311+qdd[20]*S5+qd[20]*(OMcp3_111*ROcp3_55-OMcp3_211*ROcp3_45)
    ACcp3_112 = ACcp3_111+OMcp3_211*ORcp3_312-OMcp3_311*ORcp3_212+OPcp3_211*RLcp3_312-OPcp3_311*RLcp3_212
    ACcp3_212 = ACcp3_211-OMcp3_111*ORcp3_312+OMcp3_311*ORcp3_112-OPcp3_111*RLcp3_312+OPcp3_311*RLcp3_112
    ACcp3_312 = ACcp3_311+OMcp3_111*ORcp3_212-OMcp3_211*ORcp3_112+OPcp3_111*RLcp3_212-OPcp3_211*RLcp3_112
    RLcp3_113 = ROcp3_120*s.dpt[1,17]+ROcp3_720*s.dpt[3,17]
    RLcp3_213 = ROcp3_220*s.dpt[1,17]+ROcp3_820*s.dpt[3,17]
    RLcp3_313 = ROcp3_320*s.dpt[1,17]+ROcp3_920*s.dpt[3,17]
    POcp3_113 = POcp3_112+RLcp3_113
    POcp3_213 = POcp3_212+RLcp3_213
    POcp3_313 = POcp3_312+RLcp3_313
    OMcp3_113 = OMcp3_112+ROcp3_45*qd[21]
    OMcp3_213 = OMcp3_212+ROcp3_55*qd[21]
    OMcp3_313 = OMcp3_312+qd[21]*S5
    ORcp3_113 = OMcp3_212*RLcp3_313-OMcp3_312*RLcp3_213
    ORcp3_213 = -OMcp3_112*RLcp3_313+OMcp3_312*RLcp3_113
    ORcp3_313 = OMcp3_112*RLcp3_213-OMcp3_212*RLcp3_113
    VIcp3_113 = ORcp3_113+VIcp3_112
    VIcp3_213 = ORcp3_213+VIcp3_212
    VIcp3_313 = ORcp3_313+VIcp3_312
    OPcp3_113 = OPcp3_112+ROcp3_45*qdd[21]+qd[21]*(OMcp3_212*S5-OMcp3_312*ROcp3_55)
    OPcp3_213 = OPcp3_212+ROcp3_55*qdd[21]+qd[21]*(-OMcp3_112*S5+OMcp3_312*ROcp3_45)
    OPcp3_313 = OPcp3_312+qdd[21]*S5+qd[21]*(OMcp3_112*ROcp3_55-OMcp3_212*ROcp3_45)
    ACcp3_113 = ACcp3_112+OMcp3_212*ORcp3_313-OMcp3_312*ORcp3_213+OPcp3_212*RLcp3_313-OPcp3_312*RLcp3_213
    ACcp3_213 = ACcp3_212-OMcp3_112*ORcp3_313+OMcp3_312*ORcp3_113-OPcp3_112*RLcp3_313+OPcp3_312*RLcp3_113
    ACcp3_313 = ACcp3_312+OMcp3_112*ORcp3_213-OMcp3_212*ORcp3_113+OPcp3_112*RLcp3_213-OPcp3_212*RLcp3_113
    RLcp3_114 = ROcp3_121*s.dpt[1,19]
    RLcp3_214 = ROcp3_221*s.dpt[1,19]
    RLcp3_314 = ROcp3_321*s.dpt[1,19]
    POcp3_114 = POcp3_113+RLcp3_114
    POcp3_214 = POcp3_213+RLcp3_214
    POcp3_314 = POcp3_313+RLcp3_314
    OMcp3_114 = OMcp3_113+ROcp3_45*qd[22]
    OMcp3_214 = OMcp3_213+ROcp3_55*qd[22]
    OMcp3_314 = OMcp3_313+qd[22]*S5
    ORcp3_114 = OMcp3_213*RLcp3_314-OMcp3_313*RLcp3_214
    ORcp3_214 = -OMcp3_113*RLcp3_314+OMcp3_313*RLcp3_114
    ORcp3_314 = OMcp3_113*RLcp3_214-OMcp3_213*RLcp3_114
    VIcp3_114 = ORcp3_114+VIcp3_113
    VIcp3_214 = ORcp3_214+VIcp3_213
    VIcp3_314 = ORcp3_314+VIcp3_313
    OPcp3_114 = OPcp3_113+ROcp3_45*qdd[22]+qd[22]*(OMcp3_213*S5-OMcp3_313*ROcp3_55)
    OPcp3_214 = OPcp3_213+ROcp3_55*qdd[22]+qd[22]*(-OMcp3_113*S5+OMcp3_313*ROcp3_45)
    OPcp3_314 = OPcp3_313+qdd[22]*S5+qd[22]*(OMcp3_113*ROcp3_55-OMcp3_213*ROcp3_45)
    ACcp3_114 = ACcp3_113+OMcp3_213*ORcp3_314-OMcp3_313*ORcp3_214+OPcp3_213*RLcp3_314-OPcp3_313*RLcp3_214
    ACcp3_214 = ACcp3_213-OMcp3_113*ORcp3_314+OMcp3_313*ORcp3_114-OPcp3_113*RLcp3_314+OPcp3_313*RLcp3_114
    ACcp3_314 = ACcp3_313+OMcp3_113*ORcp3_214-OMcp3_213*ORcp3_114+OPcp3_113*RLcp3_214-OPcp3_213*RLcp3_114
    sens.P[1] = POcp3_114
    sens.P[2] = POcp3_214
    sens.P[3] = POcp3_314
    sens.R[1,1] = ROcp3_122
    sens.R[1,2] = ROcp3_222
    sens.R[1,3] = ROcp3_322
    sens.R[2,1] = ROcp3_45
    sens.R[2,2] = ROcp3_55
    sens.R[2,3] = S5
    sens.R[3,1] = ROcp3_722
    sens.R[3,2] = ROcp3_822
    sens.R[3,3] = ROcp3_922
    sens.V[1] = VIcp3_114
    sens.V[2] = VIcp3_214
    sens.V[3] = VIcp3_314
    sens.OM[1] = OMcp3_114
    sens.OM[2] = OMcp3_214
    sens.OM[3] = OMcp3_314
    sens.A[1] = ACcp3_114
    sens.A[2] = ACcp3_214
    sens.A[3] = ACcp3_314
    sens.OMP[1] = OPcp3_114
    sens.OMP[2] = OPcp3_214
    sens.OMP[3] = OPcp3_314

 


# Number of continuation lines = 0


