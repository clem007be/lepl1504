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

 
# Forward Kinematics

    ALPHA33 = qdd[3]-s.g[3]
    BS14 = -qd[4]*qd[4]
    ALPHA14 = qdd[1]*C4+qdd[2]*S4
    ALPHA24 = -qdd[1]*S4+qdd[2]*C4
    OM15 = -qd[4]*S5
    OM35 = qd[4]*C5
    OMp15 = -qd[4]*qd[5]*C5-qdd[4]*S5
    OMp35 = -qd[4]*qd[5]*S5+qdd[4]*C5
    BS15 = -qd[5]*qd[5]-OM35*OM35
    BS25 = qd[5]*OM15
    BS35 = OM15*OM35
    BETA45 = BS25+OMp35
    BETA75 = -qdd[5]+BS35
    ALPHA15 = -ALPHA33*S5+C5*(ALPHA14+BS14*s.dpt[1,1])
    ALPHA25 = ALPHA24+qdd[4]*s.dpt[1,1]
    ALPHA35 = ALPHA33*C5+S5*(ALPHA14+BS14*s.dpt[1,1])
    OM16 = OM15*C6-OM35*S6
    OM26 = qd[5]+qd[6]
    OM36 = OM15*S6+OM35*C6
    OMp16 = C6*(OMp15-qd[6]*OM35)-S6*(OMp35+qd[6]*OM15)
    OMp26 = qdd[5]+qdd[6]
    OMp36 = C6*(OMp35+qd[6]*OM15)+S6*(OMp15-qd[6]*OM35)
    BS36 = OM16*OM36
    BS66 = OM26*OM36
    BS96 = -OM16*OM16-OM26*OM26
    BETA36 = BS36+OMp26
    BETA66 = BS66-OMp16
    ALPHA16 = C6*(ALPHA15+BS15*s.dpt[1,2])-S6*(ALPHA35+BETA75*s.dpt[1,2])
    ALPHA26 = ALPHA25+BETA45*s.dpt[1,2]
    ALPHA36 = C6*(ALPHA35+BETA75*s.dpt[1,2])+S6*(ALPHA15+BS15*s.dpt[1,2])
    OM17 = OM16*C7+OM26*S7
    OM27 = -OM16*S7+OM26*C7
    OM37 = qd[7]+OM36
    OMp17 = C7*(OMp16+qd[7]*OM26)+S7*(OMp26-qd[7]*OM16)
    OMp27 = C7*(OMp26-qd[7]*OM16)-S7*(OMp16+qd[7]*OM26)
    OMp37 = qdd[7]+OMp36
    ALPHA17 = C7*(ALPHA16+BETA36*s.dpt[3,3])+S7*(ALPHA26+BETA66*s.dpt[3,3])
    ALPHA27 = C7*(ALPHA26+BETA66*s.dpt[3,3])-S7*(ALPHA16+BETA36*s.dpt[3,3])
    ALPHA37 = ALPHA36+BS96*s.dpt[3,3]
    OM18 = OM17*C8-OM37*S8
    OM28 = qd[8]+OM27
    OM38 = OM17*S8+OM37*C8
    OMp18 = C8*(OMp17-qd[8]*OM37)-S8*(OMp37+qd[8]*OM17)
    OMp28 = qdd[8]+OMp27
    OMp38 = C8*(OMp37+qd[8]*OM17)+S8*(OMp17-qd[8]*OM37)
    BS38 = OM18*OM38
    BS68 = OM28*OM38
    BS98 = -OM18*OM18-OM28*OM28
    BETA38 = BS38+OMp28
    BETA68 = BS68-OMp18
    ALPHA18 = ALPHA17*C8-ALPHA37*S8
    ALPHA38 = ALPHA17*S8+ALPHA37*C8
    OM19 = OM18*C9-OM38*S9
    OM29 = qd[9]+OM28
    OM39 = OM18*S9+OM38*C9
    OMp19 = C9*(OMp18-qd[9]*OM38)-S9*(OMp38+qd[9]*OM18)
    OMp29 = qdd[9]+OMp28
    OMp39 = C9*(OMp38+qd[9]*OM18)+S9*(OMp18-qd[9]*OM38)
    BS19 = -OM29*OM29-OM39*OM39
    BS29 = OM19*OM29
    BS39 = OM19*OM39
    BS59 = -OM19*OM19-OM39*OM39
    BS69 = OM29*OM39
    BS99 = -OM19*OM19-OM29*OM29
    BETA29 = BS29-OMp39
    BETA39 = BS39+OMp29
    BETA49 = BS29+OMp39
    BETA69 = BS69-OMp19
    BETA79 = BS39-OMp29
    BETA89 = BS69+OMp19
    ALPHA19 = C9*(ALPHA18+BETA38*s.dpt[3,5])-S9*(ALPHA38+BS98*s.dpt[3,5])
    ALPHA29 = ALPHA27+BETA68*s.dpt[3,5]
    ALPHA39 = C9*(ALPHA38+BS98*s.dpt[3,5])+S9*(ALPHA18+BETA38*s.dpt[3,5])
    OM110 = OM19*C10-OM39*S10
    OM210 = qd[10]+OM29
    OM310 = OM19*S10+OM39*C10
    OMp110 = C10*(OMp19-qd[10]*OM39)-S10*(OMp39+qd[10]*OM19)
    OMp210 = qdd[10]+OMp29
    OMp310 = C10*(OMp39+qd[10]*OM19)+S10*(OMp19-qd[10]*OM39)
    BS110 = -OM210*OM210-OM310*OM310
    BS210 = OM110*OM210
    BS310 = OM110*OM310
    BETA410 = BS210+OMp310
    BETA710 = BS310-OMp210
    ALPHA110 = C10*(ALPHA19+BETA29*s.dpt[2,6]+BETA39*s.dpt[3,6]+BS19*s.dpt[1,6])-S10*(ALPHA39+BETA79*s.dpt[1,6]+BETA89*s.dpt[2,6]+BS99*s.dpt[3,6])
    ALPHA210 = ALPHA29+BETA49*s.dpt[1,6]+BETA69*s.dpt[3,6]+BS59*s.dpt[2,6]
    ALPHA310 = C10*(ALPHA39+BETA79*s.dpt[1,6]+BETA89*s.dpt[2,6]+BS99*s.dpt[3,6])+S10*(ALPHA19+BETA29*s.dpt[2,6]+BETA39*s.dpt[3,6]+BS19*s.dpt[1,6])
    OM111 = OM110*C11-OM310*S11
    OM211 = qd[11]+OM210
    OM311 = OM110*S11+OM310*C11
    OMp111 = C11*(OMp110-qd[11]*OM310)-S11*(OMp310+qd[11]*OM110)
    OMp211 = qdd[11]+OMp210
    OMp311 = C11*(OMp310+qd[11]*OM110)+S11*(OMp110-qd[11]*OM310)
    ALPHA111 = C11*(ALPHA110+BS110*s.dpt[1,11])-S11*(ALPHA310+BETA710*s.dpt[1,11])
    ALPHA211 = ALPHA210+BETA410*s.dpt[1,11]
    ALPHA311 = C11*(ALPHA310+BETA710*s.dpt[1,11])+S11*(ALPHA110+BS110*s.dpt[1,11])
    OM112 = OM19*C12-OM39*S12
    OM212 = qd[12]+OM29
    OM312 = OM19*S12+OM39*C12
    OMp112 = C12*(OMp19-qd[12]*OM39)-S12*(OMp39+qd[12]*OM19)
    OMp212 = qdd[12]+OMp29
    OMp312 = C12*(OMp39+qd[12]*OM19)+S12*(OMp19-qd[12]*OM39)
    BS112 = -OM212*OM212-OM312*OM312
    BS212 = OM112*OM212
    BS312 = OM112*OM312
    BETA412 = BS212+OMp312
    BETA712 = BS312-OMp212
    ALPHA112 = C12*(ALPHA19+BETA29*s.dpt[2,7]+BETA39*s.dpt[3,7]+BS19*s.dpt[1,7])-S12*(ALPHA39+BETA79*s.dpt[1,7]+BETA89*s.dpt[2,7]+BS99*s.dpt[3,7])
    ALPHA212 = ALPHA29+BETA49*s.dpt[1,7]+BETA69*s.dpt[3,7]+BS59*s.dpt[2,7]
    ALPHA312 = C12*(ALPHA39+BETA79*s.dpt[1,7]+BETA89*s.dpt[2,7]+BS99*s.dpt[3,7])+S12*(ALPHA19+BETA29*s.dpt[2,7]+BETA39*s.dpt[3,7]+BS19*s.dpt[1,7])
    OM113 = OM112*C13-OM312*S13
    OM213 = qd[13]+OM212
    OM313 = OM112*S13+OM312*C13
    OMp113 = C13*(OMp112-qd[13]*OM312)-S13*(OMp312+qd[13]*OM112)
    OMp213 = qdd[13]+OMp212
    OMp313 = C13*(OMp312+qd[13]*OM112)+S13*(OMp112-qd[13]*OM312)
    ALPHA113 = C13*(ALPHA112+BS112*s.dpt[1,14])-S13*(ALPHA312+BETA712*s.dpt[1,14])
    ALPHA213 = ALPHA212+BETA412*s.dpt[1,14]
    ALPHA313 = C13*(ALPHA312+BETA712*s.dpt[1,14])+S13*(ALPHA112+BS112*s.dpt[1,14])
    BS314 = OM19*OM39
    BS614 = OM29*OM39
    BS914 = -OM19*OM19-OM29*OM29
    BETA314 = BS314+OMp29
    BETA614 = BS614-OMp19
    ALPHA114 = ALPHA19+q[14]*BETA39+(2.0)*qd[14]*OM29+BS19*s.dpt[1,10]
    ALPHA214 = ALPHA29+q[14]*BETA69-(2.0)*qd[14]*OM19+BETA49*s.dpt[1,10]
    ALPHA314 = qdd[14]+ALPHA39+q[14]*BS99+BETA79*s.dpt[1,10]
    BS315 = OM16*OM36
    BS615 = OM26*OM36
    BS915 = -OM16*OM16-OM26*OM26
    BETA315 = BS315+OMp26
    BETA615 = BS615-OMp16
    ALPHA115 = ALPHA16+(2.0)*qd[15]*OM26+BETA36*Dz153
    ALPHA215 = ALPHA26-(2.0)*qd[15]*OM16+BETA66*Dz153
    ALPHA315 = qdd[15]+ALPHA36+BS96*Dz153
    OM116 = OM16*C16-OM36*S16
    OM216 = qd[16]+OM26
    OM316 = OM16*S16+OM36*C16
    OMp116 = C16*(OMp16-qd[16]*OM36)-S16*(OMp36+qd[16]*OM16)
    OMp216 = qdd[16]+OMp26
    OMp316 = C16*(OMp36+qd[16]*OM16)+S16*(OMp16-qd[16]*OM36)
    ALPHA116 = C16*(ALPHA115+BETA315*s.dpt[3,17])-S16*(ALPHA315+BS915*s.dpt[3,17])
    ALPHA216 = ALPHA215+BETA615*s.dpt[3,17]
    ALPHA316 = C16*(ALPHA315+BS915*s.dpt[3,17])+S16*(ALPHA115+BETA315*s.dpt[3,17])
 
# Backward Dynamics

    Fs116 = -s.frc[1,16]+s.m[16]*ALPHA116
    Fs216 = -s.frc[2,16]+s.m[16]*ALPHA216
    Fs316 = -s.frc[3,16]+s.m[16]*ALPHA316
    Cq116 = -s.trq[1,16]+s.In[1,16]*OMp116-s.In[5,16]*OM216*OM316+s.In[9,16]*OM216*OM316
    Cq216 = -s.trq[2,16]+s.In[1,16]*OM116*OM316+s.In[5,16]*OMp216-s.In[9,16]*OM116*OM316
    Cq316 = -s.trq[3,16]-s.In[1,16]*OM116*OM216+s.In[5,16]*OM116*OM216+s.In[9,16]*OMp316
    Fs115 = -s.frc[1,15]+s.m[15]*(ALPHA115+BETA315*s.l[3,15])
    Fs215 = -s.frc[2,15]+s.m[15]*(ALPHA215+BETA615*s.l[3,15])
    Fs315 = -s.frc[3,15]+s.m[15]*(ALPHA315+BS915*s.l[3,15])
    Fq115 = Fs115+Fs116*C16+Fs316*S16
    Fq215 = Fs215+Fs216
    Fq315 = Fs315-Fs116*S16+Fs316*C16
    Cq115 = -s.trq[1,15]+s.In[1,15]*OMp16-s.In[5,15]*OM26*OM36+s.In[9,15]*OM26*OM36+Cq116*C16+Cq316*S16-Fs215*s.l[3,15]-Fs216*s.dpt[3,17]
    Cq215 = -s.trq[2,15]+Cq216+s.In[1,15]*OM16*OM36+s.In[5,15]*OMp26-s.In[9,15]*OM16*OM36+Fs115*s.l[3,15]+s.dpt[3,17]*(Fs116*C16+Fs316*S16)
    Cq315 = -s.trq[3,15]-s.In[1,15]*OM16*OM26+s.In[5,15]*OM16*OM26+s.In[9,15]*OMp36-Cq116*S16+Cq316*C16
    Fs114 = -s.frc[1,14]+s.m[14]*(ALPHA114+BETA314*s.l[3,14])
    Fs214 = -s.frc[2,14]+s.m[14]*(ALPHA214+BETA614*s.l[3,14])
    Fs314 = -s.frc[3,14]+s.m[14]*(ALPHA314+BS914*s.l[3,14])
    Cq114 = -s.trq[1,14]+s.In[1,14]*OMp19-s.In[5,14]*OM29*OM39+s.In[9,14]*OM29*OM39-Fs214*s.l[3,14]
    Cq214 = -s.trq[2,14]+s.In[1,14]*OM19*OM39+s.In[5,14]*OMp29-s.In[9,14]*OM19*OM39+Fs114*s.l[3,14]
    Cq314 = -s.trq[3,14]-s.In[1,14]*OM19*OM29+s.In[5,14]*OM19*OM29+s.In[9,14]*OMp39
    Fs113 = -s.frc[1,13]+s.m[13]*ALPHA113
    Fs213 = -s.frc[2,13]+s.m[13]*ALPHA213
    Fs313 = -s.frc[3,13]+s.m[13]*ALPHA313
    Cq113 = -s.trq[1,13]+s.In[1,13]*OMp113-s.In[5,13]*OM213*OM313+s.In[9,13]*OM213*OM313
    Cq213 = -s.trq[2,13]+s.In[1,13]*OM113*OM313+s.In[5,13]*OMp213-s.In[9,13]*OM113*OM313
    Cq313 = -s.trq[3,13]-s.In[1,13]*OM113*OM213+s.In[5,13]*OM113*OM213+s.In[9,13]*OMp313
    Fs112 = -s.frc[1,12]+s.m[12]*(ALPHA112+BS112*s.l[1,12])
    Fs212 = -s.frc[2,12]+s.m[12]*(ALPHA212+BETA412*s.l[1,12])
    Fs312 = -s.frc[3,12]+s.m[12]*(ALPHA312+BETA712*s.l[1,12])
    Fq112 = Fs112+Fs113*C13+Fs313*S13
    Fq212 = Fs212+Fs213
    Fq312 = Fs312-Fs113*S13+Fs313*C13
    Cq112 = -s.trq[1,12]+s.In[1,12]*OMp112-s.In[5,12]*OM212*OM312+s.In[9,12]*OM212*OM312+Cq113*C13+Cq313*S13
    Cq212 = -s.trq[2,12]+Cq213+s.In[1,12]*OM112*OM312+s.In[5,12]*OMp212-s.In[9,12]*OM112*OM312-Fs312*s.l[1,12]-s.dpt[1,14]*(-Fs113*S13+Fs313*C13)
    Cq312 = -s.trq[3,12]-s.In[1,12]*OM112*OM212+s.In[5,12]*OM112*OM212+s.In[9,12]*OMp312-Cq113*S13+Cq313*C13+Fs212*s.l[1,12]+Fs213*s.dpt[1,14]
    Fs111 = -s.frc[1,11]+s.m[11]*ALPHA111
    Fs211 = -s.frc[2,11]+s.m[11]*ALPHA211
    Fs311 = -s.frc[3,11]+s.m[11]*ALPHA311
    Cq111 = -s.trq[1,11]+s.In[1,11]*OMp111-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311
    Cq211 = -s.trq[2,11]+s.In[1,11]*OM111*OM311+s.In[5,11]*OMp211-s.In[9,11]*OM111*OM311
    Cq311 = -s.trq[3,11]-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OMp311
    Fs110 = -s.frc[1,10]+s.m[10]*(ALPHA110+BS110*s.l[1,10])
    Fs210 = -s.frc[2,10]+s.m[10]*(ALPHA210+BETA410*s.l[1,10])
    Fs310 = -s.frc[3,10]+s.m[10]*(ALPHA310+BETA710*s.l[1,10])
    Fq110 = Fs110+Fs111*C11+Fs311*S11
    Fq210 = Fs210+Fs211
    Fq310 = Fs310-Fs111*S11+Fs311*C11
    Cq110 = -s.trq[1,10]+s.In[1,10]*OMp110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310+Cq111*C11+Cq311*S11
    Cq210 = -s.trq[2,10]+Cq211+s.In[1,10]*OM110*OM310+s.In[5,10]*OMp210-s.In[9,10]*OM110*OM310-Fs310*s.l[1,10]-s.dpt[1,11]*(-Fs111*S11+Fs311*C11)
    Cq310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OMp310-Cq111*S11+Cq311*C11+Fs210*s.l[1,10]+Fs211*s.dpt[1,11]
    Fs19 = -s.frc[1,9]+s.m[9]*(ALPHA19+BS19*s.l[1,9])
    Fs29 = -s.frc[2,9]+s.m[9]*(ALPHA29+BETA49*s.l[1,9])
    Fs39 = -s.frc[3,9]+s.m[9]*(ALPHA39+BETA79*s.l[1,9])
    Fq19 = Fs114+Fs19+Fq110*C10+Fq112*C12+Fq310*S10+Fq312*S12
    Fq29 = Fq210+Fq212+Fs214+Fs29
    Fq39 = Fs314+Fs39-Fq110*S10-Fq112*S12+Fq310*C10+Fq312*C12
    Cq19 = -s.trq[1,9]+Cq114-q[14]*Fs214+s.In[1,9]*OMp19-s.In[5,9]*OM29*OM39+s.In[9,9]*OM29*OM39+Cq110*C10+Cq112*C12+Cq310*S10+Cq312*S12-Fq210*s.dpt[3,6]-Fq212*s.dpt[3,7]+s.dpt[2,6]*(-Fq110*S10+Fq310*C10)+s.dpt[2,7]*(-Fq112*S12+Fq312*C12)
    Cq29 = -s.trq[2,9]+Cq210+Cq212+Cq214+q[14]*Fs114+s.In[1,9]*OM19*OM39+s.In[5,9]*OMp29-s.In[9,9]*OM19*OM39-Fs314*s.dpt[1,10]-Fs39*s.l[1,9]-s.dpt[1,6]*(-Fq110*S10+Fq310*C10)-s.dpt[1,7]*(-Fq112*S12+Fq312*C12)+s.dpt[3,6]*(Fq110*C10+Fq310*S10)+s.dpt[3,7]*(Fq112*C12+Fq312*S12)
    Cq39 = -s.trq[3,9]+Cq314-s.In[1,9]*OM19*OM29+s.In[5,9]*OM19*OM29+s.In[9,9]*OMp39-Cq110*S10-Cq112*S12+Cq310*C10+Cq312*C12+Fq210*s.dpt[1,6]+Fq212*s.dpt[1,7]+Fs214*s.dpt[1,10]+Fs29*s.l[1,9]-s.dpt[2,6]*(Fq110*C10+Fq310*S10)-s.dpt[2,7]*(Fq112*C12+Fq312*S12)
    Fs18 = -s.frc[1,8]+s.m[8]*(ALPHA18+BETA38*s.l[3,8])
    Fs28 = -s.frc[2,8]+s.m[8]*(ALPHA27+BETA68*s.l[3,8])
    Fs38 = -s.frc[3,8]+s.m[8]*(ALPHA38+BS98*s.l[3,8])
    Fq18 = Fs18+Fq19*C9+Fq39*S9
    Fq28 = Fq29+Fs28
    Fq38 = Fs38-Fq19*S9+Fq39*C9
    Cq18 = -s.trq[1,8]+s.In[1,8]*OMp18-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38+Cq19*C9+Cq39*S9-Fq29*s.dpt[3,5]-Fs28*s.l[3,8]
    Cq28 = -s.trq[2,8]+Cq29+s.In[1,8]*OM18*OM38+s.In[5,8]*OMp28-s.In[9,8]*OM18*OM38+Fs18*s.l[3,8]+s.dpt[3,5]*(Fq19*C9+Fq39*S9)
    Cq38 = -s.trq[3,8]-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OMp38-Cq19*S9+Cq39*C9
    Fq17 = Fq18*C8+Fq38*S8
    Fq37 = -Fq18*S8+Fq38*C8
    Cq17 = Cq18*C8+Cq38*S8
    Cq37 = -Cq18*S8+Cq38*C8
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA16+BETA36*s.l[3,6])
    Fs26 = -s.frc[2,6]+s.m[6]*(ALPHA26+BETA66*s.l[3,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA36+BS96*s.l[3,6])
    Fq16 = Fq115+Fs16+Fq17*C7-Fq28*S7
    Fq26 = Fq215+Fs26+Fq17*S7+Fq28*C7
    Fq36 = Fq315+Fq37+Fs36
    Cq16 = -s.trq[1,6]+Cq115+s.In[1,6]*OMp16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+Cq17*C7-Cq28*S7-Fq215*Dz153-Fs26*s.l[3,6]-s.dpt[3,3]*(Fq17*S7+Fq28*C7)
    Cq26 = -s.trq[2,6]+Cq215+s.In[1,6]*OM16*OM36+s.In[5,6]*OMp26-s.In[9,6]*OM16*OM36+Cq17*S7+Cq28*C7+Fq115*Dz153+Fs16*s.l[3,6]+s.dpt[3,3]*(Fq17*C7-Fq28*S7)
    Cq36 = -s.trq[3,6]+Cq315+Cq37-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36
    Fs15 = -s.frc[1,5]+s.m[5]*(ALPHA15+BS15*s.l[1,5])
    Fs25 = -s.frc[2,5]+s.m[5]*(ALPHA25+BETA45*s.l[1,5])
    Fs35 = -s.frc[3,5]+s.m[5]*(ALPHA35+BETA75*s.l[1,5])
    Fq15 = Fs15+Fq16*C6+Fq36*S6
    Fq25 = Fq26+Fs25
    Fq35 = Fs35-Fq16*S6+Fq36*C6
    Cq15 = -s.trq[1,5]-qd[5]*s.In[5,5]*OM35+qd[5]*s.In[9,5]*OM35+s.In[1,5]*OMp15+Cq16*C6+Cq36*S6
    Cq25 = -s.trq[2,5]+Cq26+qdd[5]*s.In[5,5]+s.In[1,5]*OM15*OM35-s.In[9,5]*OM15*OM35-Fs35*s.l[1,5]-s.dpt[1,2]*(-Fq16*S6+Fq36*C6)
    Cq35 = -s.trq[3,5]-qd[5]*s.In[1,5]*OM15+qd[5]*s.In[5,5]*OM15+s.In[9,5]*OMp35-Cq16*S6+Cq36*C6+Fq26*s.dpt[1,2]+Fs25*s.l[1,5]
    Fs14 = -s.frc[1,4]+s.m[4]*(ALPHA14+BS14*s.l[1,4])
    Fs24 = -s.frc[2,4]+s.m[4]*(ALPHA24+qdd[4]*s.l[1,4])
    Fs34 = -s.frc[3,4]+s.m[4]*ALPHA33
    Fq14 = Fs14+Fq15*C5+Fq35*S5
    Fq24 = Fq25+Fs24
    Fq34 = Fs34-Fq15*S5+Fq35*C5
    Cq34 = -s.trq[3,4]+qdd[4]*s.In[9,4]-Cq15*S5+Cq35*C5+Fq25*s.dpt[1,1]+Fs24*s.l[1,4]
    Fq13 = Fq14*C4-Fq24*S4
    Fq23 = Fq14*S4+Fq24*C4
 
# Symbolic model output

    Qq[1] = Fq13
    Qq[2] = Fq23
    Qq[3] = Fq34
    Qq[4] = Cq34
    Qq[5] = Cq25
    Qq[6] = Cq26
    Qq[7] = Cq37
    Qq[8] = Cq28
    Qq[9] = Cq29
    Qq[10] = Cq210
    Qq[11] = Cq211
    Qq[12] = Cq212
    Qq[13] = Cq213
    Qq[14] = Fs314
    Qq[15] = Fq315
    Qq[16] = Cq216

# Number of continuation lines = 0


