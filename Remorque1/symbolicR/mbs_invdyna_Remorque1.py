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
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S16 = sin(q[16])
    C16 = cos(q[16])
 
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
    BS17 = -OM27*OM27-OM37*OM37
    BS27 = OM17*OM27
    BS37 = OM17*OM37
    BETA47 = BS27+OMp37
    BETA77 = BS37-OMp27
    ALPHA17 = C7*(ALPHA16+BETA36*s.dpt[3,1]+BS16*s.dpt[1,1])-S7*(ALPHA36+BETA76*s.dpt[1,1]+BS96*s.dpt[3,1])
    ALPHA27 = ALPHA25+BETA46*s.dpt[1,1]+BETA66*s.dpt[3,1]
    ALPHA37 = C7*(ALPHA36+BETA76*s.dpt[1,1]+BS96*s.dpt[3,1])+S7*(ALPHA16+BETA36*s.dpt[3,1]+BS16*s.dpt[1,1])
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
    ALPHA18 = C8*(ALPHA17+BS17*s.dpt[1,2])-S8*(ALPHA37+BETA77*s.dpt[1,2])
    ALPHA28 = ALPHA27+BETA47*s.dpt[1,2]
    ALPHA38 = C8*(ALPHA37+BETA77*s.dpt[1,2])+S8*(ALPHA17+BS17*s.dpt[1,2])
    OM19 = OM18*C9-OM38*S9
    OM29 = qd[9]+OM28
    OM39 = OM18*S9+OM38*C9
    OMp19 = C9*(OMp18-qd[9]*OM38)-S9*(OMp38+qd[9]*OM18)
    OMp29 = qdd[9]+OMp28
    OMp39 = C9*(OMp38+qd[9]*OM18)+S9*(OMp18-qd[9]*OM38)
    ALPHA19 = C9*(ALPHA18+BETA38*s.dpt[3,3])-S9*(ALPHA38+BS98*s.dpt[3,3])
    ALPHA29 = ALPHA28+BETA68*s.dpt[3,3]
    ALPHA39 = C9*(ALPHA38+BS98*s.dpt[3,3])+S9*(ALPHA18+BETA38*s.dpt[3,3])
    OM110 = OM18*C10+OM28*S10
    OM210 = -OM18*S10+OM28*C10
    OM310 = qd[10]+OM38
    OMp110 = C10*(OMp18+qd[10]*OM28)+S10*(OMp28-qd[10]*OM18)
    OMp210 = C10*(OMp28-qd[10]*OM18)-S10*(OMp18+qd[10]*OM28)
    OMp310 = qdd[10]+OMp38
    ALPHA110 = C10*(ALPHA18+BETA38*s.dpt[3,4])+S10*(ALPHA28+BETA68*s.dpt[3,4])
    ALPHA210 = C10*(ALPHA28+BETA68*s.dpt[3,4])-S10*(ALPHA18+BETA38*s.dpt[3,4])
    ALPHA310 = ALPHA38+BS98*s.dpt[3,4]
    OM111 = OM110*C11-OM310*S11
    OM211 = qd[11]+OM210
    OM311 = OM110*S11+OM310*C11
    OMp111 = C11*(OMp110-qd[11]*OM310)-S11*(OMp310+qd[11]*OM110)
    OMp211 = qdd[11]+OMp210
    OMp311 = C11*(OMp310+qd[11]*OM110)+S11*(OMp110-qd[11]*OM310)
    BS311 = OM111*OM311
    BS611 = OM211*OM311
    BS911 = -OM111*OM111-OM211*OM211
    BETA311 = BS311+OMp211
    BETA611 = BS611-OMp111
    ALPHA111 = ALPHA110*C11-ALPHA310*S11
    ALPHA311 = ALPHA110*S11+ALPHA310*C11
    OM112 = OM111*C12-OM311*S12
    OM212 = qd[12]+OM211
    OM312 = OM111*S12+OM311*C12
    OMp112 = C12*(OMp111-qd[12]*OM311)-S12*(OMp311+qd[12]*OM111)
    OMp212 = qdd[12]+OMp211
    OMp312 = C12*(OMp311+qd[12]*OM111)+S12*(OMp111-qd[12]*OM311)
    BS112 = -OM212*OM212-OM312*OM312
    BS212 = OM112*OM212
    BS312 = OM112*OM312
    BS512 = -OM112*OM112-OM312*OM312
    BS612 = OM212*OM312
    BS912 = -OM112*OM112-OM212*OM212
    BETA212 = BS212-OMp312
    BETA312 = BS312+OMp212
    BETA412 = BS212+OMp312
    BETA612 = BS612-OMp112
    BETA712 = BS312-OMp212
    BETA812 = BS612+OMp112
    ALPHA112 = C12*(ALPHA111+BETA311*s.dpt[3,6])-S12*(ALPHA311+BS911*s.dpt[3,6])
    ALPHA212 = ALPHA210+BETA611*s.dpt[3,6]
    ALPHA312 = C12*(ALPHA311+BS911*s.dpt[3,6])+S12*(ALPHA111+BETA311*s.dpt[3,6])
    OM113 = OM112*C13-OM312*S13
    OM213 = qd[13]+OM212
    OM313 = OM112*S13+OM312*C13
    OMp113 = C13*(OMp112-qd[13]*OM312)-S13*(OMp312+qd[13]*OM112)
    OMp213 = qdd[13]+OMp212
    OMp313 = C13*(OMp312+qd[13]*OM112)+S13*(OMp112-qd[13]*OM312)
    BS113 = -OM213*OM213-OM313*OM313
    BS213 = OM113*OM213
    BS313 = OM113*OM313
    BETA413 = BS213+OMp313
    BETA713 = BS313-OMp213
    ALPHA113 = C13*(ALPHA112+BETA212*s.dpt[2,7]+BETA312*s.dpt[3,7]+BS112*s.dpt[1,7])-S13*(ALPHA312+BETA712*s.dpt[1,7]+BETA812*s.dpt[2,7]+BS912*s.dpt[3,7])
    ALPHA213 = ALPHA212+BETA412*s.dpt[1,7]+BETA612*s.dpt[3,7]+BS512*s.dpt[2,7]
    ALPHA313 = C13*(ALPHA312+BETA712*s.dpt[1,7]+BETA812*s.dpt[2,7]+BS912*s.dpt[3,7])+S13*(ALPHA112+BETA212*s.dpt[2,7]+BETA312*s.dpt[3,7]+BS112*s.dpt[1,7])
    OM114 = OM113*C14-OM313*S14
    OM214 = qd[14]+OM213
    OM314 = OM113*S14+OM313*C14
    OMp114 = C14*(OMp113-qd[14]*OM313)-S14*(OMp313+qd[14]*OM113)
    OMp214 = qdd[14]+OMp213
    OMp314 = C14*(OMp313+qd[14]*OM113)+S14*(OMp113-qd[14]*OM313)
    ALPHA114 = C14*(ALPHA113+BS113*s.dpt[1,12])-S14*(ALPHA313+BETA713*s.dpt[1,12])
    ALPHA214 = ALPHA213+BETA413*s.dpt[1,12]
    ALPHA314 = C14*(ALPHA313+BETA713*s.dpt[1,12])+S14*(ALPHA113+BS113*s.dpt[1,12])
    OM115 = OM112*C15-OM312*S15
    OM215 = qd[15]+OM212
    OM315 = OM112*S15+OM312*C15
    OMp115 = C15*(OMp112-qd[15]*OM312)-S15*(OMp312+qd[15]*OM112)
    OMp215 = qdd[15]+OMp212
    OMp315 = C15*(OMp312+qd[15]*OM112)+S15*(OMp112-qd[15]*OM312)
    BS115 = -OM215*OM215-OM315*OM315
    BS215 = OM115*OM215
    BS315 = OM115*OM315
    BETA415 = BS215+OMp315
    BETA715 = BS315-OMp215
    ALPHA115 = C15*(ALPHA112+BETA212*s.dpt[2,8]+BETA312*s.dpt[3,8]+BS112*s.dpt[1,8])-S15*(ALPHA312+BETA712*s.dpt[1,8]+BETA812*s.dpt[2,8]+BS912*s.dpt[3,8])
    ALPHA215 = ALPHA212+BETA412*s.dpt[1,8]+BETA612*s.dpt[3,8]+BS512*s.dpt[2,8]
    ALPHA315 = C15*(ALPHA312+BETA712*s.dpt[1,8]+BETA812*s.dpt[2,8]+BS912*s.dpt[3,8])+S15*(ALPHA112+BETA212*s.dpt[2,8]+BETA312*s.dpt[3,8]+BS112*s.dpt[1,8])
    OM116 = OM115*C16-OM315*S16
    OM216 = qd[16]+OM215
    OM316 = OM115*S16+OM315*C16
    OMp116 = C16*(OMp115-qd[16]*OM315)-S16*(OMp315+qd[16]*OM115)
    OMp216 = qdd[16]+OMp215
    OMp316 = C16*(OMp315+qd[16]*OM115)+S16*(OMp115-qd[16]*OM315)
    ALPHA116 = C16*(ALPHA115+BS115*s.dpt[1,15])-S16*(ALPHA315+BETA715*s.dpt[1,15])
    ALPHA216 = ALPHA215+BETA415*s.dpt[1,15]
    ALPHA316 = C16*(ALPHA315+BETA715*s.dpt[1,15])+S16*(ALPHA115+BS115*s.dpt[1,15])
    BS317 = OM112*OM312
    BS617 = OM212*OM312
    BS917 = -OM112*OM112-OM212*OM212
    BETA317 = BS317+OMp212
    BETA617 = BS617-OMp112
    ALPHA117 = ALPHA112+q[17]*BETA312+(2.0)*qd[17]*OM212+BS112*s.dpt[1,11]
    ALPHA217 = ALPHA212+q[17]*BETA612-(2.0)*qd[17]*OM112+BETA412*s.dpt[1,11]
    ALPHA317 = qdd[17]+ALPHA312+q[17]*BS912+BETA712*s.dpt[1,11]
 
# Backward Dynamics

    Fs117 = -s.frc[1,17]+s.m[17]*(ALPHA117+BETA317*s.l[3,17])
    Fs217 = -s.frc[2,17]+s.m[17]*(ALPHA217+BETA617*s.l[3,17])
    Fs317 = -s.frc[3,17]+s.m[17]*(ALPHA317+BS917*s.l[3,17])
    Cq117 = -s.trq[1,17]+s.In[1,17]*OMp112-s.In[5,17]*OM212*OM312+s.In[9,17]*OM212*OM312-Fs217*s.l[3,17]
    Cq217 = -s.trq[2,17]+s.In[1,17]*OM112*OM312+s.In[5,17]*OMp212-s.In[9,17]*OM112*OM312+Fs117*s.l[3,17]
    Cq317 = -s.trq[3,17]-s.In[1,17]*OM112*OM212+s.In[5,17]*OM112*OM212+s.In[9,17]*OMp312
    Fs116 = -s.frc[1,16]+s.m[16]*ALPHA116
    Fs216 = -s.frc[2,16]+s.m[16]*ALPHA216
    Fs316 = -s.frc[3,16]+s.m[16]*ALPHA316
    Cq116 = -s.trq[1,16]+s.In[1,16]*OMp116-s.In[5,16]*OM216*OM316+s.In[9,16]*OM216*OM316
    Cq216 = -s.trq[2,16]+s.In[1,16]*OM116*OM316+s.In[5,16]*OMp216-s.In[9,16]*OM116*OM316
    Cq316 = -s.trq[3,16]-s.In[1,16]*OM116*OM216+s.In[5,16]*OM116*OM216+s.In[9,16]*OMp316
    Fs115 = -s.frc[1,15]+s.m[15]*(ALPHA115+BS115*s.l[1,15])
    Fs215 = -s.frc[2,15]+s.m[15]*(ALPHA215+BETA415*s.l[1,15])
    Fs315 = -s.frc[3,15]+s.m[15]*(ALPHA315+BETA715*s.l[1,15])
    Fq115 = Fs115+Fs116*C16+Fs316*S16
    Fq215 = Fs215+Fs216
    Fq315 = Fs315-Fs116*S16+Fs316*C16
    Cq115 = -s.trq[1,15]+s.In[1,15]*OMp115-s.In[5,15]*OM215*OM315+s.In[9,15]*OM215*OM315+Cq116*C16+Cq316*S16
    Cq215 = -s.trq[2,15]+Cq216+s.In[1,15]*OM115*OM315+s.In[5,15]*OMp215-s.In[9,15]*OM115*OM315-Fs315*s.l[1,15]-s.dpt[1,15]*(-Fs116*S16+Fs316*C16)
    Cq315 = -s.trq[3,15]-s.In[1,15]*OM115*OM215+s.In[5,15]*OM115*OM215+s.In[9,15]*OMp315-Cq116*S16+Cq316*C16+Fs215*s.l[1,15]+Fs216*s.dpt[1,15]
    Fs114 = -s.frc[1,14]+s.m[14]*ALPHA114
    Fs214 = -s.frc[2,14]+s.m[14]*ALPHA214
    Fs314 = -s.frc[3,14]+s.m[14]*ALPHA314
    Cq114 = -s.trq[1,14]+s.In[1,14]*OMp114-s.In[5,14]*OM214*OM314+s.In[9,14]*OM214*OM314
    Cq214 = -s.trq[2,14]+s.In[1,14]*OM114*OM314+s.In[5,14]*OMp214-s.In[9,14]*OM114*OM314
    Cq314 = -s.trq[3,14]-s.In[1,14]*OM114*OM214+s.In[5,14]*OM114*OM214+s.In[9,14]*OMp314
    Fs113 = -s.frc[1,13]+s.m[13]*(ALPHA113+BS113*s.l[1,13])
    Fs213 = -s.frc[2,13]+s.m[13]*(ALPHA213+BETA413*s.l[1,13])
    Fs313 = -s.frc[3,13]+s.m[13]*(ALPHA313+BETA713*s.l[1,13])
    Fq113 = Fs113+Fs114*C14+Fs314*S14
    Fq213 = Fs213+Fs214
    Fq313 = Fs313-Fs114*S14+Fs314*C14
    Cq113 = -s.trq[1,13]+s.In[1,13]*OMp113-s.In[5,13]*OM213*OM313+s.In[9,13]*OM213*OM313+Cq114*C14+Cq314*S14
    Cq213 = -s.trq[2,13]+Cq214+s.In[1,13]*OM113*OM313+s.In[5,13]*OMp213-s.In[9,13]*OM113*OM313-Fs313*s.l[1,13]-s.dpt[1,12]*(-Fs114*S14+Fs314*C14)
    Cq313 = -s.trq[3,13]-s.In[1,13]*OM113*OM213+s.In[5,13]*OM113*OM213+s.In[9,13]*OMp313-Cq114*S14+Cq314*C14+Fs213*s.l[1,13]+Fs214*s.dpt[1,12]
    Fs112 = -s.frc[1,12]+s.m[12]*(ALPHA112+BS112*s.l[1,12])
    Fs212 = -s.frc[2,12]+s.m[12]*(ALPHA212+BETA412*s.l[1,12])
    Fs312 = -s.frc[3,12]+s.m[12]*(ALPHA312+BETA712*s.l[1,12])
    Fq112 = Fs112+Fs117+Fq113*C13+Fq115*C15+Fq313*S13+Fq315*S15
    Fq212 = Fq213+Fq215+Fs212+Fs217
    Fq312 = Fs312+Fs317-Fq113*S13-Fq115*S15+Fq313*C13+Fq315*C15
    Cq112 = -s.trq[1,12]+Cq117-q[17]*Fs217+s.In[1,12]*OMp112-s.In[5,12]*OM212*OM312+s.In[9,12]*OM212*OM312+Cq113*C13+Cq115*C15+Cq313*S13+Cq315*S15-Fq213*s.dpt[3,7]-Fq215*s.dpt[3,8]+s.dpt[2,7]*(-Fq113*S13+Fq313*C13)+s.dpt[2,8]*(-Fq115*S15+Fq315*C15)
    Cq212 = -s.trq[2,12]+Cq213+Cq215+Cq217+q[17]*Fs117+s.In[1,12]*OM112*OM312+s.In[5,12]*OMp212-s.In[9,12]*OM112*OM312-Fs312*s.l[1,12]-Fs317*s.dpt[1,11]-s.dpt[1,7]*(-Fq113*S13+Fq313*C13)-s.dpt[1,8]*(-Fq115*S15+Fq315*C15)+s.dpt[3,7]*(Fq113*C13+Fq313*S13)+s.dpt[3,8]*(Fq115*C15+Fq315*S15)
    Cq312 = -s.trq[3,12]+Cq317-s.In[1,12]*OM112*OM212+s.In[5,12]*OM112*OM212+s.In[9,12]*OMp312-Cq113*S13-Cq115*S15+Cq313*C13+Cq315*C15+Fq213*s.dpt[1,7]+Fq215*s.dpt[1,8]+Fs212*s.l[1,12]+Fs217*s.dpt[1,11]-s.dpt[2,7]*(Fq113*C13+Fq313*S13)-s.dpt[2,8]*(Fq115*C15+Fq315*S15)
    Fs111 = -s.frc[1,11]+s.m[11]*(ALPHA111+BETA311*s.l[3,11])
    Fs211 = -s.frc[2,11]+s.m[11]*(ALPHA210+BETA611*s.l[3,11])
    Fs311 = -s.frc[3,11]+s.m[11]*(ALPHA311+BS911*s.l[3,11])
    Fq111 = Fs111+Fq112*C12+Fq312*S12
    Fq211 = Fq212+Fs211
    Fq311 = Fs311-Fq112*S12+Fq312*C12
    Cq111 = -s.trq[1,11]+s.In[1,11]*OMp111-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311+Cq112*C12+Cq312*S12-Fq212*s.dpt[3,6]-Fs211*s.l[3,11]
    Cq211 = -s.trq[2,11]+Cq212+s.In[1,11]*OM111*OM311+s.In[5,11]*OMp211-s.In[9,11]*OM111*OM311+Fs111*s.l[3,11]+s.dpt[3,6]*(Fq112*C12+Fq312*S12)
    Cq311 = -s.trq[3,11]-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OMp311-Cq112*S12+Cq312*C12
    Fq110 = Fq111*C11+Fq311*S11
    Fq310 = -Fq111*S11+Fq311*C11
    Cq110 = Cq111*C11+Cq311*S11
    Cq310 = -Cq111*S11+Cq311*C11
    Fs19 = -s.frc[1,9]+s.m[9]*ALPHA19
    Fs29 = -s.frc[2,9]+s.m[9]*ALPHA29
    Fs39 = -s.frc[3,9]+s.m[9]*ALPHA39
    Cq19 = -s.trq[1,9]+s.In[1,9]*OMp19-s.In[5,9]*OM29*OM39+s.In[9,9]*OM29*OM39
    Cq29 = -s.trq[2,9]+s.In[1,9]*OM19*OM39+s.In[5,9]*OMp29-s.In[9,9]*OM19*OM39
    Cq39 = -s.trq[3,9]-s.In[1,9]*OM19*OM29+s.In[5,9]*OM19*OM29+s.In[9,9]*OMp39
    Fs18 = -s.frc[1,8]+s.m[8]*(ALPHA18+BETA38*s.l[3,8])
    Fs28 = -s.frc[2,8]+s.m[8]*(ALPHA28+BETA68*s.l[3,8])
    Fs38 = -s.frc[3,8]+s.m[8]*(ALPHA38+BS98*s.l[3,8])
    Fq18 = Fs18+Fq110*C10-Fq211*S10+Fs19*C9+Fs39*S9
    Fq28 = Fs28+Fs29+Fq110*S10+Fq211*C10
    Fq38 = Fq310+Fs38-Fs19*S9+Fs39*C9
    Cq18 = -s.trq[1,8]+s.In[1,8]*OMp18-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38+Cq110*C10+Cq19*C9-Cq211*S10+Cq39*S9-Fs28*s.l[3,8]-Fs29*s.dpt[3,3]-s.dpt[3,4]*(Fq110*S10+Fq211*C10)
    Cq28 = -s.trq[2,8]+Cq29+s.In[1,8]*OM18*OM38+s.In[5,8]*OMp28-s.In[9,8]*OM18*OM38+Cq110*S10+Cq211*C10+Fs18*s.l[3,8]+s.dpt[3,3]*(Fs19*C9+Fs39*S9)+s.dpt[3,4]*(Fq110*C10-Fq211*S10)
    Cq38 = -s.trq[3,8]+Cq310-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OMp38-Cq19*S9+Cq39*C9
    Fs17 = -s.frc[1,7]+s.m[7]*(ALPHA17+BS17*s.l[1,7])
    Fs27 = -s.frc[2,7]+s.m[7]*(ALPHA27+BETA47*s.l[1,7])
    Fs37 = -s.frc[3,7]+s.m[7]*(ALPHA37+BETA77*s.l[1,7])
    Fq17 = Fs17+Fq18*C8+Fq38*S8
    Fq27 = Fq28+Fs27
    Fq37 = Fs37-Fq18*S8+Fq38*C8
    Cq17 = -s.trq[1,7]+s.In[1,7]*OMp17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37+Cq18*C8+Cq38*S8
    Cq27 = -s.trq[2,7]+Cq28+s.In[1,7]*OM17*OM37+s.In[5,7]*OMp27-s.In[9,7]*OM17*OM37-Fs37*s.l[1,7]-s.dpt[1,2]*(-Fq18*S8+Fq38*C8)
    Cq37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OMp37-Cq18*S8+Cq38*C8+Fq28*s.dpt[1,2]+Fs27*s.l[1,7]
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA16+BETA36*s.l[3,6]+BS16*s.l[1,6])
    Fs26 = -s.frc[2,6]+s.m[6]*(ALPHA25+BETA46*s.l[1,6]+BETA66*s.l[3,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA36+BETA76*s.l[1,6]+BS96*s.l[3,6])
    Fq16 = Fs16+Fq17*C7+Fq37*S7
    Fq26 = Fq27+Fs26
    Fq36 = Fs36-Fq17*S7+Fq37*C7
    Cq16 = -s.trq[1,6]+s.In[1,6]*OMp16+s.In[3,6]*OMp36-s.In[5,6]*OM26*OM36+Cq17*C7+Cq37*S7-Fq27*s.dpt[3,1]-Fs26*s.l[3,6]+OM26*(s.In[3,6]*OM16+s.In[9,6]*OM36)
    Cq26 = -s.trq[2,6]+Cq27+s.In[5,6]*OMp26+Fs16*s.l[3,6]-Fs36*s.l[1,6]-OM16*(s.In[3,6]*OM16+s.In[9,6]*OM36)+OM36*(s.In[1,6]*OM16+s.In[3,6]*OM36)-s.dpt[1,1]*(-Fq17*S7+Fq37*C7)+s.dpt[3,1]*(Fq17*C7+Fq37*S7)
    Cq36 = -s.trq[3,6]+s.In[3,6]*OMp16+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36-Cq17*S7+Cq37*C7+Fq27*s.dpt[1,1]+Fs26*s.l[1,6]-OM26*(s.In[1,6]*OM16+s.In[3,6]*OM36)
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
    Qq[9] = Cq29
    Qq[10] = Cq310
    Qq[11] = Cq211
    Qq[12] = Cq212
    Qq[13] = Cq213
    Qq[14] = Cq214
    Qq[15] = Cq215
    Qq[16] = Cq216
    Qq[17] = Fs317

# Number of continuation lines = 0


