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

  S2 = sin(q[2])
  C2 = cos(q[2])
  S3 = sin(q[3])
  C3 = cos(q[3])
  S4 = sin(q[4])
  C4 = cos(q[4])
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

    ROcp1_13 = C2*C3-S2*S3
    ROcp1_33 = -C2*S3-S2*C3
    ROcp1_73 = C2*S3+S2*C3
    ROcp1_93 = C2*C3-S2*S3
    ROcp1_14 = ROcp1_13*C4-ROcp1_73*S4
    ROcp1_34 = ROcp1_33*C4-ROcp1_93*S4
    ROcp1_74 = ROcp1_13*S4+ROcp1_73*C4
    ROcp1_94 = ROcp1_33*S4+ROcp1_93*C4
    POcp1_32 = q[1]+s.dpt[3,1]
    RLcp1_13 = s.dpt[1,5]*C2+s.dpt[3,5]*S2
    RLcp1_33 = -s.dpt[1,5]*S2+s.dpt[3,5]*C2
    POcp1_13 = RLcp1_13+s.dpt[1,1]
    POcp1_33 = POcp1_32+RLcp1_33
    OMcp1_23 = qd[2]+qd[3]
    ORcp1_13 = RLcp1_33*qd[2]
    ORcp1_33 = -RLcp1_13*qd[2]
    VIcp1_33 = ORcp1_33+qd[1]
    OPcp1_23 = qdd[2]+qdd[3]
    ACcp1_13 = ORcp1_33*qd[2]+RLcp1_33*qdd[2]
    ACcp1_33 = qdd[1]-ORcp1_13*qd[2]-RLcp1_13*qdd[2]
    RLcp1_14 = ROcp1_73*s.dpt[3,6]
    RLcp1_34 = ROcp1_93*s.dpt[3,6]
    POcp1_14 = POcp1_13+RLcp1_14
    POcp1_34 = POcp1_33+RLcp1_34
    OMcp1_24 = OMcp1_23+qd[4]
    ORcp1_14 = OMcp1_23*RLcp1_34
    ORcp1_34 = -OMcp1_23*RLcp1_14
    VIcp1_14 = ORcp1_13+ORcp1_14
    VIcp1_34 = ORcp1_34+VIcp1_33
    OPcp1_24 = OPcp1_23+qdd[4]
    ACcp1_14 = ACcp1_13+OMcp1_23*ORcp1_34+OPcp1_23*RLcp1_34
    ACcp1_34 = ACcp1_33-OMcp1_23*ORcp1_14-OPcp1_23*RLcp1_14
    sens.P[1] = POcp1_14
    sens.P[2] = 0
    sens.P[3] = POcp1_34
    sens.R[1,1] = ROcp1_14
    sens.R[1,3] = ROcp1_34
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp1_74
    sens.R[3,3] = ROcp1_94
    sens.V[1] = VIcp1_14
    sens.V[2] = 0
    sens.V[3] = VIcp1_34
    sens.OM[1] = 0
    sens.OM[2] = OMcp1_24
    sens.OM[3] = 0
    sens.A[1] = ACcp1_14
    sens.A[2] = 0
    sens.A[3] = ACcp1_34
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp1_24
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


