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
  S5 = sin(q[5])
  C5 = cos(q[5])
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

    sens.P[1] = 0
    sens.P[2] = 0
    sens.P[3] = q[1]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = qd[1]
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[3,1] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = qdd[1]
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 2): 

    POcp2_32 = q[1]+s.dpt[3,1]
    sens.P[1] = s.dpt[1,1]
    sens.P[2] = 0
    sens.P[3] = POcp2_32
    sens.R[1,1] = C2
    sens.R[1,3] = -S2
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S2
    sens.R[3,3] = C2
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = qd[1]
    sens.OM[1] = 0
    sens.OM[2] = qd[2]
    sens.OM[3] = 0
    sens.J[3,1] = (1.0)
    sens.J[5,2] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = qdd[1]
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[2]
    sens.OMP[3] = 0

  if (isens == 3): 

    ROcp3_13 = C2*C3-S2*S3
    ROcp3_33 = -C2*S3-S2*C3
    ROcp3_73 = C2*S3+S2*C3
    ROcp3_93 = C2*C3-S2*S3
    POcp3_32 = q[1]+s.dpt[3,1]
    RLcp3_13 = s.dpt[1,5]*C2+s.dpt[3,5]*S2
    RLcp3_33 = -s.dpt[1,5]*S2+s.dpt[3,5]*C2
    POcp3_13 = RLcp3_13+s.dpt[1,1]
    POcp3_33 = POcp3_32+RLcp3_33
    OMcp3_23 = qd[2]+qd[3]
    ORcp3_13 = RLcp3_33*qd[2]
    ORcp3_33 = -RLcp3_13*qd[2]
    VIcp3_33 = ORcp3_33+qd[1]
    OPcp3_23 = qdd[2]+qdd[3]
    ACcp3_13 = ORcp3_33*qd[2]+RLcp3_33*qdd[2]
    ACcp3_33 = qdd[1]-ORcp3_13*qd[2]-RLcp3_13*qdd[2]
    sens.P[1] = POcp3_13
    sens.P[2] = 0
    sens.P[3] = POcp3_33
    sens.R[1,1] = ROcp3_13
    sens.R[1,3] = ROcp3_33
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp3_73
    sens.R[3,3] = ROcp3_93
    sens.V[1] = ORcp3_13
    sens.V[2] = 0
    sens.V[3] = VIcp3_33
    sens.OM[1] = 0
    sens.OM[2] = OMcp3_23
    sens.OM[3] = 0
    sens.J[1,2] = RLcp3_33
    sens.J[3,1] = (1.0)
    sens.J[3,2] = -RLcp3_13
    sens.J[5,2] = (1.0)
    sens.J[5,3] = (1.0)
    sens.A[1] = ACcp3_13
    sens.A[2] = 0
    sens.A[3] = ACcp3_33
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp3_23
    sens.OMP[3] = 0

  if (isens == 4): 

    ROcp4_13 = C2*C3-S2*S3
    ROcp4_33 = -C2*S3-S2*C3
    ROcp4_73 = C2*S3+S2*C3
    ROcp4_93 = C2*C3-S2*S3
    ROcp4_14 = ROcp4_13*C4-ROcp4_73*S4
    ROcp4_34 = ROcp4_33*C4-ROcp4_93*S4
    ROcp4_74 = ROcp4_13*S4+ROcp4_73*C4
    ROcp4_94 = ROcp4_33*S4+ROcp4_93*C4
    POcp4_32 = q[1]+s.dpt[3,1]
    RLcp4_13 = s.dpt[1,5]*C2+s.dpt[3,5]*S2
    RLcp4_33 = -s.dpt[1,5]*S2+s.dpt[3,5]*C2
    POcp4_13 = RLcp4_13+s.dpt[1,1]
    POcp4_33 = POcp4_32+RLcp4_33
    OMcp4_23 = qd[2]+qd[3]
    ORcp4_13 = RLcp4_33*qd[2]
    ORcp4_33 = -RLcp4_13*qd[2]
    VIcp4_33 = ORcp4_33+qd[1]
    OPcp4_23 = qdd[2]+qdd[3]
    ACcp4_13 = ORcp4_33*qd[2]+RLcp4_33*qdd[2]
    ACcp4_33 = qdd[1]-ORcp4_13*qd[2]-RLcp4_13*qdd[2]
    RLcp4_14 = ROcp4_73*s.dpt[3,6]
    RLcp4_34 = ROcp4_93*s.dpt[3,6]
    POcp4_14 = POcp4_13+RLcp4_14
    POcp4_34 = POcp4_33+RLcp4_34
    JTcp4_14_2 = RLcp4_33+RLcp4_34
    JTcp4_34_2 = -RLcp4_13-RLcp4_14
    OMcp4_24 = OMcp4_23+qd[4]
    ORcp4_14 = OMcp4_23*RLcp4_34
    ORcp4_34 = -OMcp4_23*RLcp4_14
    VIcp4_14 = ORcp4_13+ORcp4_14
    VIcp4_34 = ORcp4_34+VIcp4_33
    OPcp4_24 = OPcp4_23+qdd[4]
    ACcp4_14 = ACcp4_13+OMcp4_23*ORcp4_34+OPcp4_23*RLcp4_34
    ACcp4_34 = ACcp4_33-OMcp4_23*ORcp4_14-OPcp4_23*RLcp4_14
    sens.P[1] = POcp4_14
    sens.P[2] = 0
    sens.P[3] = POcp4_34
    sens.R[1,1] = ROcp4_14
    sens.R[1,3] = ROcp4_34
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp4_74
    sens.R[3,3] = ROcp4_94
    sens.V[1] = VIcp4_14
    sens.V[2] = 0
    sens.V[3] = VIcp4_34
    sens.OM[1] = 0
    sens.OM[2] = OMcp4_24
    sens.OM[3] = 0
    sens.J[1,2] = JTcp4_14_2
    sens.J[1,3] = RLcp4_34
    sens.J[3,1] = (1.0)
    sens.J[3,2] = JTcp4_34_2
    sens.J[3,3] = -RLcp4_14
    sens.J[5,2] = (1.0)
    sens.J[5,3] = (1.0)
    sens.J[5,4] = (1.0)
    sens.A[1] = ACcp4_14
    sens.A[2] = 0
    sens.A[3] = ACcp4_34
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp4_24
    sens.OMP[3] = 0

  if (isens == 5): 

    POcp5_32 = q[1]+s.dpt[3,3]
    sens.P[1] = s.dpt[1,3]
    sens.P[2] = 0
    sens.P[3] = POcp5_32
    sens.R[1,1] = C5
    sens.R[1,3] = -S5
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S5
    sens.R[3,3] = C5
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = qd[1]
    sens.OM[1] = 0
    sens.OM[2] = qd[5]
    sens.OM[3] = 0
    sens.J[3,1] = (1.0)
    sens.J[5,5] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = qdd[1]
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[5]
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


