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
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
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

 
# Forward Kinematics

    ALPHA31 = qdd[1]-s.g[3]
    BS12 = -qd[2]*qd[2]
    BS92 = -qd[2]*qd[2]
    ALPHA12 = -ALPHA31*S2
    ALPHA32 = ALPHA31*C2
    OM23 = qd[2]+qd[3]
    OMp23 = qdd[2]+qdd[3]
    BS13 = -OM23*OM23
    BS93 = -OM23*OM23
    ALPHA13 = C3*(ALPHA12+qdd[2]*s.dpt[3,5]+BS12*s.dpt[1,5])-S3*(ALPHA32-qdd[2]*s.dpt[1,5]+BS92*s.dpt[3,5])
    ALPHA33 = C3*(ALPHA32-qdd[2]*s.dpt[1,5]+BS92*s.dpt[3,5])+S3*(ALPHA12+qdd[2]*s.dpt[3,5]+BS12*s.dpt[1,5])
    OMp24 = qdd[4]+OMp23
    ALPHA14 = C4*(ALPHA13+OMp23*s.dpt[3,6])-S4*(ALPHA33+BS93*s.dpt[3,6])
    ALPHA34 = C4*(ALPHA33+BS93*s.dpt[3,6])+S4*(ALPHA13+OMp23*s.dpt[3,6])
    BS15 = -qd[5]*qd[5]
    BS95 = -qd[5]*qd[5]
    ALPHA15 = -ALPHA31*S5
    ALPHA35 = ALPHA31*C5
 
# Backward Dynamics

    Fs15 = -s.frc[1,5]+s.m[5]*(ALPHA15+qdd[5]*s.l[3,5]+BS15*s.l[1,5])
    Fs35 = -s.frc[3,5]+s.m[5]*(ALPHA35-qdd[5]*s.l[1,5]+BS95*s.l[3,5])
    Cq25 = -s.trq[2,5]+qdd[5]*s.In[5,5]+Fs15*s.l[3,5]-Fs35*s.l[1,5]
    Fs14 = -s.frc[1,4]+s.m[4]*ALPHA14
    Fs34 = -s.frc[3,4]+s.m[4]*ALPHA34
    Cq24 = -s.trq[2,4]+s.In[5,4]*OMp24
    Fs13 = -s.frc[1,3]+s.m[3]*(ALPHA13+BS13*s.l[1,3]+OMp23*s.l[3,3])
    Fs33 = -s.frc[3,3]+s.m[3]*(ALPHA33+BS93*s.l[3,3]-OMp23*s.l[1,3])
    Fq13 = Fs13+Fs14*C4+Fs34*S4
    Fq33 = Fs33-Fs14*S4+Fs34*C4
    Cq23 = -s.trq[2,3]+Cq24+s.In[5,3]*OMp23+Fs13*s.l[3,3]-Fs33*s.l[1,3]+s.dpt[3,6]*(Fs14*C4+Fs34*S4)
    Fs12 = -s.frc[1,2]+s.m[2]*(ALPHA12+qdd[2]*s.l[3,2]+BS12*s.l[1,2])
    Fs32 = -s.frc[3,2]+s.m[2]*(ALPHA32-qdd[2]*s.l[1,2]+BS92*s.l[3,2])
    Fq12 = Fs12+Fq13*C3+Fq33*S3
    Fq32 = Fs32-Fq13*S3+Fq33*C3
    Cq22 = -s.trq[2,2]+Cq23+qdd[2]*s.In[5,2]+Fs12*s.l[3,2]-Fs32*s.l[1,2]-s.dpt[1,5]*(-Fq13*S3+Fq33*C3)+s.dpt[3,5]*(Fq13*C3+Fq33*S3)
    Fs31 = -s.frc[3,1]+s.m[1]*ALPHA31
    Fq31 = Fs31-Fq12*S2+Fq32*C2-Fs15*S5+Fs35*C5
 
# Symbolic model output

    Qq[1] = Fq31
    Qq[2] = Cq22
    Qq[3] = Cq23
    Qq[4] = Cq24
    Qq[5] = Cq25

# Number of continuation lines = 0


