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

 
# Trigonometric functions

    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
    S4 = sin(q[4])
    C4 = cos(q[4])
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics

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
    ORcp1_13 = qd[2]*RLcp1_33
    ORcp1_33 = -qd[2]*RLcp1_13
    VIcp1_33 = qd[1]+ORcp1_33
    OPcp1_23 = qdd[2]+qdd[3]
    ACcp1_13 = qd[2]*ORcp1_33+qdd[2]*RLcp1_33
    ACcp1_33 = qdd[1]-qd[2]*ORcp1_13-qdd[2]*RLcp1_13
    RLcp1_14 = ROcp1_73*s.dpt[3,6]
    RLcp1_34 = ROcp1_93*s.dpt[3,6]
    POcp1_14 = POcp1_13+RLcp1_14
    POcp1_34 = POcp1_33+RLcp1_34
    OMcp1_24 = qd[4]+OMcp1_23
    ORcp1_14 = OMcp1_23*RLcp1_34
    ORcp1_34 = -OMcp1_23*RLcp1_14
    VIcp1_14 = ORcp1_13+ORcp1_14
    VIcp1_34 = ORcp1_34+VIcp1_33
    OPcp1_24 = qdd[4]+OPcp1_23
    ACcp1_14 = ACcp1_13+OMcp1_23*ORcp1_34+OPcp1_23*RLcp1_34
    ACcp1_34 = ACcp1_33-OMcp1_23*ORcp1_14-OPcp1_23*RLcp1_14
    PxF1[1] = POcp1_14
    PxF1[2] = 0
    PxF1[3] = POcp1_34
    RxF1[1,1] = ROcp1_14
    RxF1[1,3] = ROcp1_34
    RxF1[2,2] = (1.0)
    RxF1[3,1] = ROcp1_74
    RxF1[3,3] = ROcp1_94
    VxF1[1] = VIcp1_14
    VxF1[2] = 0
    VxF1[3] = VIcp1_34
    OMxF1[1] = 0
    OMxF1[2] = OMcp1_24
    OMxF1[3] = 0
    AxF1[1] = ACcp1_14
    AxF1[2] = 0
    AxF1[3] = ACcp1_34
    OMPxF1[1] = 0
    OMPxF1[2] = OPcp1_24
    OMPxF1[3] = 0
 
# Sensor Forces 

    SWr1 = s.user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1)
    xfrc11 = RxF1[1,1]*SWr1[1]+RxF1[1,3]*SWr1[3]
    xfrc21 = RxF1[2,2]*SWr1[2]
    xfrc31 = RxF1[3,1]*SWr1[1]+RxF1[3,3]*SWr1[3]
    xtrq11 = RxF1[1,1]*SWr1[4]+RxF1[1,3]*SWr1[6]
    xtrq21 = RxF1[2,2]*SWr1[5]
    xtrq31 = RxF1[3,1]*SWr1[4]+RxF1[3,3]*SWr1[6]
    trqext_1_4_0 = xtrq11-xfrc21*SWr1[9]+xfrc31*SWr1[8]
    trqext_2_4_0 = xtrq21+xfrc11*SWr1[9]-xfrc31*SWr1[7]
    trqext_3_4_0 = xtrq31-xfrc11*SWr1[8]+xfrc21*SWr1[7]
 
# Symbolic model output

    frc[1,4] = s.frc[1,4]+xfrc11
    frc[2,4] = s.frc[2,4]+xfrc21
    frc[3,4] = s.frc[3,4]+xfrc31
    trq[1,4] = s.trq[1,4]+trqext_1_4_0
    trq[2,4] = s.trq[2,4]+trqext_2_4_0
    trq[3,4] = s.trq[3,4]+trqext_3_4_0

# Number of continuation lines = 0


