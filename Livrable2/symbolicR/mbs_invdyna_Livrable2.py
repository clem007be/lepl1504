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
#	==> Generation Date: Thu Mar 16 17:54:00 2023
#
#	==> Project name: Livrable2
#
#	==> Number of joints: 10
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

 
# Forward Kinematics

    ALPHA33 = qdd[3]-s.g[3]
    ALPHA14 = qdd[1]*C4+qdd[2]*S4
    ALPHA24 = -qdd[1]*S4+qdd[2]*C4
    OM25 = qd[4]*S5
    OM35 = qd[4]*C5
    OMp25 = qd[4]*qd[5]*C5+qdd[4]*S5
    OMp35 = -qd[4]*qd[5]*S5+qdd[4]*C5
    ALPHA25 = ALPHA24*C5+ALPHA33*S5
    ALPHA35 = -ALPHA24*S5+ALPHA33*C5
    OM16 = qd[5]*C6-OM35*S6
    OM26 = qd[6]+OM25
    OM36 = qd[5]*S6+OM35*C6
    OMp16 = C6*(qdd[5]-qd[6]*OM35)-S6*(OMp35+qd[5]*qd[6])
    OMp26 = qdd[6]+OMp25
    OMp36 = C6*(OMp35+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM35)
    BS16 = -OM26*OM26-OM36*OM36
    BS26 = OM16*OM26
    BS36 = OM16*OM36
    BS66 = OM26*OM36
    BS96 = -OM16*OM16-OM26*OM26
    BETA36 = BS36+OMp26
    BETA46 = BS26+OMp36
    BETA66 = BS66-OMp16
    BETA76 = BS36-OMp26
    ALPHA16 = ALPHA14*C6-ALPHA35*S6
    ALPHA36 = ALPHA14*S6+ALPHA35*C6
    OM17 = OM16*C7-OM36*S7
    OM27 = qd[7]+OM26
    OM37 = OM16*S7+OM36*C7
    OMp17 = C7*(OMp16-qd[7]*OM36)-S7*(OMp36+qd[7]*OM16)
    OMp27 = qdd[7]+OMp26
    OMp37 = C7*(OMp36+qd[7]*OM16)+S7*(OMp16-qd[7]*OM36)
    ALPHA17 = C7*(ALPHA16+BS16*s.dpt[1,1])-S7*(ALPHA36+BETA76*s.dpt[1,1])
    ALPHA27 = ALPHA25+BETA46*s.dpt[1,1]
    ALPHA37 = C7*(ALPHA36+BETA76*s.dpt[1,1])+S7*(ALPHA16+BS16*s.dpt[1,1])
    OM18 = OM16*C8-OM36*S8
    OM28 = qd[8]+OM26
    OM38 = OM16*S8+OM36*C8
    OMp18 = C8*(OMp16-qd[8]*OM36)-S8*(OMp36+qd[8]*OM16)
    OMp28 = qdd[8]+OMp26
    OMp38 = C8*(OMp36+qd[8]*OM16)+S8*(OMp16-qd[8]*OM36)
    ALPHA18 = C8*(ALPHA16+BETA36*s.dpt[3,2]+BS16*s.dpt[1,2])-S8*(ALPHA36+BETA76*s.dpt[1,2]+BS96*s.dpt[3,2])
    ALPHA28 = ALPHA25+BETA46*s.dpt[1,2]+BETA66*s.dpt[3,2]
    ALPHA38 = C8*(ALPHA36+BETA76*s.dpt[1,2]+BS96*s.dpt[3,2])+S8*(ALPHA16+BETA36*s.dpt[3,2]+BS16*s.dpt[1,2])
    OM19 = OM18*C9+OM28*S9
    OM29 = -OM18*S9+OM28*C9
    OM39 = qd[9]+OM38
    OMp19 = C9*(OMp18+qd[9]*OM28)+S9*(OMp28-qd[9]*OM18)
    OMp29 = C9*(OMp28-qd[9]*OM18)-S9*(OMp18+qd[9]*OM28)
    OMp39 = qdd[9]+OMp38
    BS39 = OM19*OM39
    BS69 = OM29*OM39
    BS99 = -OM19*OM19-OM29*OM29
    BETA39 = BS39+OMp29
    BETA69 = BS69-OMp19
    ALPHA19 = ALPHA18*C9+ALPHA28*S9
    ALPHA29 = -ALPHA18*S9+ALPHA28*C9
    OM110 = OM19*C10-OM39*S10
    OM210 = qd[10]+OM29
    OM310 = OM19*S10+OM39*C10
    OMp110 = C10*(OMp19-qd[10]*OM39)-S10*(OMp39+qd[10]*OM19)
    OMp210 = qdd[10]+OMp29
    OMp310 = C10*(OMp39+qd[10]*OM19)+S10*(OMp19-qd[10]*OM39)
    ALPHA110 = C10*(ALPHA19+BETA39*s.dpt[3,4])-S10*(ALPHA38+BS99*s.dpt[3,4])
    ALPHA210 = ALPHA29+BETA69*s.dpt[3,4]
    ALPHA310 = C10*(ALPHA38+BS99*s.dpt[3,4])+S10*(ALPHA19+BETA39*s.dpt[3,4])
 
# Backward Dynamics

    Fs110 = -s.frc[1,10]+s.m[10]*ALPHA110
    Fs210 = -s.frc[2,10]+s.m[10]*ALPHA210
    Fs310 = -s.frc[3,10]+s.m[10]*ALPHA310
    Cq110 = -s.trq[1,10]+s.In[1,10]*OMp110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310
    Cq210 = -s.trq[2,10]+s.In[1,10]*OM110*OM310+s.In[5,10]*OMp210-s.In[9,10]*OM110*OM310
    Cq310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OMp310
    Fq19 = -s.frc[1,9]+Fs110*C10+Fs310*S10
    Fq29 = -s.frc[2,9]+Fs210
    Fq39 = -s.frc[3,9]-Fs110*S10+Fs310*C10
    Cq19 = -s.trq[1,9]+Cq110*C10+Cq310*S10-Fs210*s.dpt[3,4]
    Cq29 = -s.trq[2,9]+Cq210+s.dpt[3,4]*(Fs110*C10+Fs310*S10)
    Cq39 = -s.trq[3,9]-Cq110*S10+Cq310*C10
    Fq18 = Fq19*C9-Fq29*S9
    Fq28 = Fq19*S9+Fq29*C9
    Cq18 = Cq19*C9-Cq29*S9
    Cq28 = Cq19*S9+Cq29*C9
    Fs17 = -s.frc[1,7]+s.m[7]*ALPHA17
    Fs27 = -s.frc[2,7]+s.m[7]*ALPHA27
    Fs37 = -s.frc[3,7]+s.m[7]*ALPHA37
    Cq17 = -s.trq[1,7]+s.In[1,7]*OMp17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37
    Cq27 = -s.trq[2,7]+s.In[1,7]*OM17*OM37+s.In[5,7]*OMp27-s.In[9,7]*OM17*OM37
    Cq37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OMp37
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA16+BETA36*s.l[3,6])
    Fs26 = -s.frc[2,6]+s.m[6]*(ALPHA25+BETA66*s.l[3,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA36+BS96*s.l[3,6])
    Fq16 = Fs16+Fq18*C8+Fq39*S8+Fs17*C7+Fs37*S7
    Fq26 = Fq28+Fs26+Fs27
    Fq36 = Fs36-Fq18*S8+Fq39*C8-Fs17*S7+Fs37*C7
    Cq16 = -s.trq[1,6]+s.In[1,6]*OMp16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+Cq17*C7+Cq18*C8+Cq37*S7+Cq39*S8-Fq28*s.dpt[3,2]-Fs26*s.l[3,6]
    Cq26 = -s.trq[2,6]+Cq27+Cq28+s.In[1,6]*OM16*OM36+s.In[5,6]*OMp26-s.In[9,6]*OM16*OM36+Fs16*s.l[3,6]-s.dpt[1,1]*(-Fs17*S7+Fs37*C7)-s.dpt[1,2]*(-Fq18*S8+Fq39*C8)+s.dpt[3,2]*(Fq18*C8+Fq39*S8)
    Cq36 = -s.trq[3,6]-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36-Cq17*S7-Cq18*S8+Cq37*C7+Cq39*C8+Fq28*s.dpt[1,2]+Fs27*s.dpt[1,1]
    Fq15 = Fq16*C6+Fq36*S6
    Fq35 = -Fq16*S6+Fq36*C6
    Cq15 = Cq16*C6+Cq36*S6
    Cq35 = -Cq16*S6+Cq36*C6
    Fq24 = Fq26*C5-Fq35*S5
    Fq34 = Fq26*S5+Fq35*C5
    Cq34 = Cq26*S5+Cq35*C5
    Fq13 = Fq15*C4-Fq24*S4
    Fq23 = Fq15*S4+Fq24*C4
 
# Symbolic model output

    Qq[1] = Fq13
    Qq[2] = Fq23
    Qq[3] = Fq34
    Qq[4] = Cq34
    Qq[5] = Cq15
    Qq[6] = Cq26
    Qq[7] = Cq27
    Qq[8] = Cq28
    Qq[9] = Cq39
    Qq[10] = Cq210

# Number of continuation lines = 0


