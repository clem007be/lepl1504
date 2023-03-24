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

 
# Forward Kinematics

    ALPHA14 = qdd[1]*C4+qdd[2]*S4
    ALPHA24 = -qdd[1]*S4+qdd[2]*C4
    OM25 = qd[4]*S5
    OM35 = qd[4]*C5
    OMp25 = qd[4]*qd[5]*C5+qdd[4]*S5
    OMp35 = -qd[4]*qd[5]*S5+qdd[4]*C5
    ALPHA25 = qdd[3]*S5+ALPHA24*C5
    ALPHA35 = qdd[3]*C5-ALPHA24*S5
    OM16 = qd[5]*C6-OM35*S6
    OM26 = qd[6]+OM25
    OM36 = qd[5]*S6+OM35*C6
    OMp16 = C6*(qdd[5]-qd[6]*OM35)-S6*(OMp35+qd[5]*qd[6])
    OMp26 = qdd[6]+OMp25
    OMp36 = C6*(OMp35+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM35)
    BS16 = -OM26*OM26-OM36*OM36
    BS26 = OM16*OM26
    BS36 = OM16*OM36
    BETA46 = BS26+OMp36
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
    ALPHA17 = C7*(ALPHA16+BS16*s.dpt[1,1])-S7*(ALPHA36+BETA76*s.dpt[1,1])
    ALPHA27 = ALPHA25+BETA46*s.dpt[1,1]
    ALPHA37 = C7*(ALPHA36+BETA76*s.dpt[1,1])+S7*(ALPHA16+BS16*s.dpt[1,1])
    OM18 = OM17*C8-OM37*S8
    OM28 = qd[8]+OM27
    OM38 = OM17*S8+OM37*C8
    OMp18 = C8*(OMp17-qd[8]*OM37)-S8*(OMp37+qd[8]*OM17)
    OMp28 = qdd[8]+OMp27
    OMp38 = C8*(OMp37+qd[8]*OM17)+S8*(OMp17-qd[8]*OM37)
    BS18 = -OM28*OM28-OM38*OM38
    BS28 = OM18*OM28
    BS38 = OM18*OM38
    BETA48 = BS28+OMp38
    BETA78 = BS38-OMp28
    ALPHA18 = C8*(ALPHA17+BS17*s.dpt[1,2])-S8*(ALPHA37+BETA77*s.dpt[1,2])
    ALPHA28 = ALPHA27+BETA47*s.dpt[1,2]
    ALPHA38 = C8*(ALPHA37+BETA77*s.dpt[1,2])+S8*(ALPHA17+BS17*s.dpt[1,2])
    OM19 = OM18*C9-OM38*S9
    OM29 = qd[9]+OM28
    OM39 = OM18*S9+OM38*C9
    OMp19 = C9*(OMp18-qd[9]*OM38)-S9*(OMp38+qd[9]*OM18)
    OMp29 = qdd[9]+OMp28
    OMp39 = C9*(OMp38+qd[9]*OM18)+S9*(OMp18-qd[9]*OM38)
    BS19 = -OM29*OM29-OM39*OM39
    BS29 = OM19*OM29
    BS39 = OM19*OM39
    BETA49 = BS29+OMp39
    BETA79 = BS39-OMp29
    ALPHA19 = C9*(ALPHA18+BS18*s.dpt[1,3])-S9*(ALPHA38+BETA78*s.dpt[1,3])
    ALPHA29 = ALPHA28+BETA48*s.dpt[1,3]
    ALPHA39 = C9*(ALPHA38+BETA78*s.dpt[1,3])+S9*(ALPHA18+BS18*s.dpt[1,3])
    BS110 = -OM29*OM29-OM39*OM39
    BS210 = OM19*OM29
    BS310 = OM19*OM39
    BETA410 = BS210+OMp39
    BETA710 = BS310-OMp29
    ALPHA110 = qdd[10]+ALPHA19+BS19*Dz101
    ALPHA210 = ALPHA29+(2.0)*qd[10]*OM39+BETA49*Dz101
    ALPHA310 = ALPHA39-(2.0)*qd[10]*OM29+BETA79*Dz101
    OM111 = OM19*C11-OM39*S11
    OM211 = qd[11]+OM29
    OM311 = OM19*S11+OM39*C11
    OMp111 = C11*(OMp19-qd[11]*OM39)-S11*(OMp39+qd[11]*OM19)
    OMp211 = qdd[11]+OMp29
    OMp311 = C11*(OMp39+qd[11]*OM19)+S11*(OMp19-qd[11]*OM39)
    BS111 = -OM211*OM211-OM311*OM311
    BS211 = OM111*OM211
    BS311 = OM111*OM311
    BS511 = -OM111*OM111-OM311*OM311
    BS611 = OM211*OM311
    BS911 = -OM111*OM111-OM211*OM211
    BETA211 = BS211-OMp311
    BETA311 = BS311+OMp211
    BETA411 = BS211+OMp311
    BETA611 = BS611-OMp111
    BETA711 = BS311-OMp211
    BETA811 = BS611+OMp111
    ALPHA111 = C11*(ALPHA110+BS110*s.dpt[1,5])-S11*(ALPHA310+BETA710*s.dpt[1,5])
    ALPHA211 = ALPHA210+BETA410*s.dpt[1,5]
    ALPHA311 = C11*(ALPHA310+BETA710*s.dpt[1,5])+S11*(ALPHA110+BS110*s.dpt[1,5])
    OM112 = OM111*C12-OM311*S12
    OM212 = qd[12]+OM211
    OM312 = OM111*S12+OM311*C12
    OMp112 = C12*(OMp111-qd[12]*OM311)-S12*(OMp311+qd[12]*OM111)
    OMp212 = qdd[12]+OMp211
    OMp312 = C12*(OMp311+qd[12]*OM111)+S12*(OMp111-qd[12]*OM311)
    ALPHA112 = ALPHA111*C12-ALPHA311*S12
    ALPHA312 = ALPHA111*S12+ALPHA311*C12
    OM113 = OM112*C13+OM212*S13
    OM213 = -OM112*S13+OM212*C13
    OM313 = qd[13]+OM312
    OMp113 = C13*(OMp112+qd[13]*OM212)+S13*(OMp212-qd[13]*OM112)
    OMp213 = C13*(OMp212-qd[13]*OM112)-S13*(OMp112+qd[13]*OM212)
    OMp313 = qdd[13]+OMp312
    BS313 = OM113*OM313
    BS613 = OM213*OM313
    BS913 = -OM113*OM113-OM213*OM213
    BETA313 = BS313+OMp213
    BETA613 = BS613-OMp113
    ALPHA113 = ALPHA112*C13+ALPHA211*S13
    ALPHA213 = -ALPHA112*S13+ALPHA211*C13
    BS314 = OM113*OM313
    BS614 = OM213*OM313
    BS914 = -OM113*OM113-OM213*OM213
    BETA314 = BS314+OMp213
    BETA614 = BS614-OMp113
    ALPHA114 = ALPHA113+(2.0)*qd[14]*OM213+BETA313*Dz143
    ALPHA214 = ALPHA213-(2.0)*qd[14]*OM113+BETA613*Dz143
    ALPHA314 = qdd[14]+ALPHA312+BS913*Dz143
    OM115 = OM113*C15-OM313*S15
    OM215 = qd[15]+OM213
    OM315 = OM113*S15+OM313*C15
    OMp115 = C15*(OMp113-qd[15]*OM313)-S15*(OMp313+qd[15]*OM113)
    OMp215 = qdd[15]+OMp213
    OMp315 = C15*(OMp313+qd[15]*OM113)+S15*(OMp113-qd[15]*OM313)
    BS116 = -OM211*OM211-OM311*OM311
    BS216 = OM111*OM211
    BS316 = OM111*OM311
    BS616 = OM211*OM311
    BS916 = -OM111*OM111-OM211*OM211
    BETA316 = BS316+OMp211
    BETA416 = BS216+OMp311
    BETA616 = BS616-OMp111
    BETA716 = BS316-OMp211
    ALPHA116 = qdd[16]+ALPHA111+q[16]*BS111
    ALPHA216 = ALPHA211+q[16]*BETA411+(2.0)*qd[16]*OM311
    ALPHA316 = ALPHA311+q[16]*BETA711-(2.0)*qd[16]*OM211
    OM117 = OM111*C17-OM311*S17
    OM217 = qd[17]+OM211
    OM317 = OM111*S17+OM311*C17
    OMp117 = C17*(OMp111-qd[17]*OM311)-S17*(OMp311+qd[17]*OM111)
    OMp217 = qdd[17]+OMp211
    OMp317 = C17*(OMp311+qd[17]*OM111)+S17*(OMp111-qd[17]*OM311)
    BS117 = -OM217*OM217-OM317*OM317
    BS217 = OM117*OM217
    BS317 = OM117*OM317
    BS617 = OM217*OM317
    BS917 = -OM117*OM117-OM217*OM217
    BETA317 = BS317+OMp217
    BETA417 = BS217+OMp317
    BETA617 = BS617-OMp117
    BETA717 = BS317-OMp217
    ALPHA117 = C17*(ALPHA111+BETA211*s.dpt[2,6]+BETA311*s.dpt[3,6])-S17*(ALPHA311+BETA811*s.dpt[2,6]+BS911*s.dpt[3,6])
    ALPHA217 = ALPHA211+BETA611*s.dpt[3,6]+BS511*s.dpt[2,6]
    ALPHA317 = C17*(ALPHA311+BETA811*s.dpt[2,6]+BS911*s.dpt[3,6])+S17*(ALPHA111+BETA211*s.dpt[2,6]+BETA311*s.dpt[3,6])
    OM118 = OM117*C18-OM317*S18
    OM218 = qd[18]+OM217
    OM318 = OM117*S18+OM317*C18
    OMp118 = C18*(OMp117-qd[18]*OM317)-S18*(OMp317+qd[18]*OM117)
    OMp218 = qdd[18]+OMp217
    OMp318 = C18*(OMp317+qd[18]*OM117)+S18*(OMp117-qd[18]*OM317)
    BS118 = -OM218*OM218-OM318*OM318
    BS218 = OM118*OM218
    BS318 = OM118*OM318
    BETA418 = BS218+OMp318
    BETA718 = BS318-OMp218
    ALPHA118 = C18*(ALPHA117+BETA317*s.dpt[3,13]+BS117*s.dpt[1,13])-S18*(ALPHA317+BETA717*s.dpt[1,13]+BS917*s.dpt[3,13])
    ALPHA218 = ALPHA217+BETA417*s.dpt[1,13]+BETA617*s.dpt[3,13]
    ALPHA318 = C18*(ALPHA317+BETA717*s.dpt[1,13]+BS917*s.dpt[3,13])+S18*(ALPHA117+BETA317*s.dpt[3,13]+BS117*s.dpt[1,13])
    OM119 = OM118*C19-OM318*S19
    OM219 = qd[19]+OM218
    OM319 = OM118*S19+OM318*C19
    OMp119 = C19*(OMp118-qd[19]*OM318)-S19*(OMp318+qd[19]*OM118)
    OMp219 = qdd[19]+OMp218
    OMp319 = C19*(OMp318+qd[19]*OM118)+S19*(OMp118-qd[19]*OM318)
    OM120 = OM111*C20-OM311*S20
    OM220 = qd[20]+OM211
    OM320 = OM111*S20+OM311*C20
    OMp120 = C20*(OMp111-qd[20]*OM311)-S20*(OMp311+qd[20]*OM111)
    OMp220 = qdd[20]+OMp211
    OMp320 = C20*(OMp311+qd[20]*OM111)+S20*(OMp111-qd[20]*OM311)
    BS120 = -OM220*OM220-OM320*OM320
    BS220 = OM120*OM220
    BS320 = OM120*OM320
    BS620 = OM220*OM320
    BS920 = -OM120*OM120-OM220*OM220
    BETA320 = BS320+OMp220
    BETA420 = BS220+OMp320
    BETA620 = BS620-OMp120
    BETA720 = BS320-OMp220
    ALPHA120 = C20*(ALPHA111+BETA211*s.dpt[2,7]+BETA311*s.dpt[3,7])-S20*(ALPHA311+BETA811*s.dpt[2,7]+BS911*s.dpt[3,7])
    ALPHA220 = ALPHA211+BETA611*s.dpt[3,7]+BS511*s.dpt[2,7]
    ALPHA320 = C20*(ALPHA311+BETA811*s.dpt[2,7]+BS911*s.dpt[3,7])+S20*(ALPHA111+BETA211*s.dpt[2,7]+BETA311*s.dpt[3,7])
    OM121 = OM120*C21-OM320*S21
    OM221 = qd[21]+OM220
    OM321 = OM120*S21+OM320*C21
    OMp121 = C21*(OMp120-qd[21]*OM320)-S21*(OMp320+qd[21]*OM120)
    OMp221 = qdd[21]+OMp220
    OMp321 = C21*(OMp320+qd[21]*OM120)+S21*(OMp120-qd[21]*OM320)
    BS121 = -OM221*OM221-OM321*OM321
    BS221 = OM121*OM221
    BS321 = OM121*OM321
    BETA421 = BS221+OMp321
    BETA721 = BS321-OMp221
    ALPHA121 = C21*(ALPHA120+BETA320*s.dpt[3,17]+BS120*s.dpt[1,17])-S21*(ALPHA320+BETA720*s.dpt[1,17]+BS920*s.dpt[3,17])
    ALPHA221 = ALPHA220+BETA420*s.dpt[1,17]+BETA620*s.dpt[3,17]
    ALPHA321 = C21*(ALPHA320+BETA720*s.dpt[1,17]+BS920*s.dpt[3,17])+S21*(ALPHA120+BETA320*s.dpt[3,17]+BS120*s.dpt[1,17])
    OM122 = OM121*C22-OM321*S22
    OM222 = qd[22]+OM221
    OM322 = OM121*S22+OM321*C22
    OMp122 = C22*(OMp121-qd[22]*OM321)-S22*(OMp321+qd[22]*OM121)
    OMp222 = qdd[22]+OMp221
    OMp322 = C22*(OMp321+qd[22]*OM121)+S22*(OMp121-qd[22]*OM321)
 
# Backward Dynamics

    Cq122 = -s.trq[1,22]+s.In[1,22]*OMp122-s.In[5,22]*OM222*OM322+s.In[9,22]*OM222*OM322
    Cq222 = -s.trq[2,22]+s.In[1,22]*OM122*OM322+s.In[5,22]*OMp222-s.In[9,22]*OM122*OM322
    Cq322 = -s.trq[3,22]-s.In[1,22]*OM122*OM222+s.In[5,22]*OM122*OM222+s.In[9,22]*OMp322
    Fs121 = -s.frc[1,21]+s.m[21]*(ALPHA121+BS121*s.l[1,21])
    Fs221 = -s.frc[2,21]+s.m[21]*(ALPHA221+BETA421*s.l[1,21])
    Fs321 = -s.frc[3,21]+s.m[21]*(ALPHA321+BETA721*s.l[1,21])
    Fq121 = Fs121-s.frc[1,22]*C22-s.frc[3,22]*S22
    Fq221 = -s.frc[2,22]+Fs221
    Fq321 = Fs321+s.frc[1,22]*S22-s.frc[3,22]*C22
    Cq121 = -s.trq[1,21]+s.In[1,21]*OMp121-s.In[5,21]*OM221*OM321+s.In[9,21]*OM221*OM321+Cq122*C22+Cq322*S22
    Cq221 = -s.trq[2,21]+Cq222+s.In[1,21]*OM121*OM321+s.In[5,21]*OMp221-s.In[9,21]*OM121*OM321-Fs321*s.l[1,21]-s.dpt[1,19]*(s.frc[1,22]*S22-s.frc[3,22]*C22)
    Cq321 = -s.trq[3,21]-s.In[1,21]*OM121*OM221+s.In[5,21]*OM121*OM221+s.In[9,21]*OMp321-s.frc[2,22]*s.dpt[1,19]-Cq122*S22+Cq322*C22+Fs221*s.l[1,21]
    Fs120 = -s.frc[1,20]+s.m[20]*(ALPHA120+BS120*s.l[1,20])
    Fs220 = -s.frc[2,20]+s.m[20]*(ALPHA220+BETA420*s.l[1,20])
    Fs320 = -s.frc[3,20]+s.m[20]*(ALPHA320+BETA720*s.l[1,20])
    Fq120 = Fs120+Fq121*C21+Fq321*S21
    Fq220 = Fq221+Fs220
    Fq320 = Fs320-Fq121*S21+Fq321*C21
    Cq120 = -s.trq[1,20]+s.In[1,20]*OMp120-s.In[5,20]*OM220*OM320+s.In[9,20]*OM220*OM320+Cq121*C21+Cq321*S21-Fq221*s.dpt[3,17]
    Cq220 = -s.trq[2,20]+Cq221+s.In[1,20]*OM120*OM320+s.In[5,20]*OMp220-s.In[9,20]*OM120*OM320-Fs320*s.l[1,20]-s.dpt[1,17]*(-Fq121*S21+Fq321*C21)+s.dpt[3,17]*(Fq121*C21+Fq321*S21)
    Cq320 = -s.trq[3,20]-s.In[1,20]*OM120*OM220+s.In[5,20]*OM120*OM220+s.In[9,20]*OMp320-Cq121*S21+Cq321*C21+Fq221*s.dpt[1,17]+Fs220*s.l[1,20]
    Cq119 = -s.trq[1,19]+s.In[1,19]*OMp119-s.In[5,19]*OM219*OM319+s.In[9,19]*OM219*OM319
    Cq219 = -s.trq[2,19]+s.In[1,19]*OM119*OM319+s.In[5,19]*OMp219-s.In[9,19]*OM119*OM319
    Cq319 = -s.trq[3,19]-s.In[1,19]*OM119*OM219+s.In[5,19]*OM119*OM219+s.In[9,19]*OMp319
    Fs118 = -s.frc[1,18]+s.m[18]*(ALPHA118+BS118*s.l[1,18])
    Fs218 = -s.frc[2,18]+s.m[18]*(ALPHA218+BETA418*s.l[1,18])
    Fs318 = -s.frc[3,18]+s.m[18]*(ALPHA318+BETA718*s.l[1,18])
    Fq118 = Fs118-s.frc[1,19]*C19-s.frc[3,19]*S19
    Fq218 = -s.frc[2,19]+Fs218
    Fq318 = Fs318+s.frc[1,19]*S19-s.frc[3,19]*C19
    Cq118 = -s.trq[1,18]+s.In[1,18]*OMp118-s.In[5,18]*OM218*OM318+s.In[9,18]*OM218*OM318+Cq119*C19+Cq319*S19
    Cq218 = -s.trq[2,18]+Cq219+s.In[1,18]*OM118*OM318+s.In[5,18]*OMp218-s.In[9,18]*OM118*OM318-Fs318*s.l[1,18]-s.dpt[1,15]*(s.frc[1,19]*S19-s.frc[3,19]*C19)
    Cq318 = -s.trq[3,18]-s.In[1,18]*OM118*OM218+s.In[5,18]*OM118*OM218+s.In[9,18]*OMp318-s.frc[2,19]*s.dpt[1,15]-Cq119*S19+Cq319*C19+Fs218*s.l[1,18]
    Fs117 = -s.frc[1,17]+s.m[17]*(ALPHA117+BS117*s.l[1,17])
    Fs217 = -s.frc[2,17]+s.m[17]*(ALPHA217+BETA417*s.l[1,17])
    Fs317 = -s.frc[3,17]+s.m[17]*(ALPHA317+BETA717*s.l[1,17])
    Fq117 = Fs117+Fq118*C18+Fq318*S18
    Fq217 = Fq218+Fs217
    Fq317 = Fs317-Fq118*S18+Fq318*C18
    Cq117 = -s.trq[1,17]+s.In[1,17]*OMp117-s.In[5,17]*OM217*OM317+s.In[9,17]*OM217*OM317+Cq118*C18+Cq318*S18-Fq218*s.dpt[3,13]
    Cq217 = -s.trq[2,17]+Cq218+s.In[1,17]*OM117*OM317+s.In[5,17]*OMp217-s.In[9,17]*OM117*OM317-Fs317*s.l[1,17]-s.dpt[1,13]*(-Fq118*S18+Fq318*C18)+s.dpt[3,13]*(Fq118*C18+Fq318*S18)
    Cq317 = -s.trq[3,17]-s.In[1,17]*OM117*OM217+s.In[5,17]*OM117*OM217+s.In[9,17]*OMp317-Cq118*S18+Cq318*C18+Fq218*s.dpt[1,13]+Fs217*s.l[1,17]
    Fs116 = -s.frc[1,16]+s.m[16]*(ALPHA116+BETA316*s.l[3,16]+BS116*s.l[1,16])
    Fs216 = -s.frc[2,16]+s.m[16]*(ALPHA216+BETA416*s.l[1,16]+BETA616*s.l[3,16])
    Fs316 = -s.frc[3,16]+s.m[16]*(ALPHA316+BETA716*s.l[1,16]+BS916*s.l[3,16])
    Cq116 = -s.trq[1,16]+s.In[1,16]*OMp111-s.In[5,16]*OM211*OM311+s.In[9,16]*OM211*OM311-Fs216*s.l[3,16]
    Cq216 = -s.trq[2,16]+s.In[1,16]*OM111*OM311+s.In[5,16]*OMp211-s.In[9,16]*OM111*OM311+Fs116*s.l[3,16]-Fs316*s.l[1,16]
    Cq316 = -s.trq[3,16]-s.In[1,16]*OM111*OM211+s.In[5,16]*OM111*OM211+s.In[9,16]*OMp311+Fs216*s.l[1,16]
    Cq115 = -s.trq[1,15]+s.In[1,15]*OMp115-s.In[5,15]*OM215*OM315+s.In[9,15]*OM215*OM315
    Cq215 = -s.trq[2,15]+s.In[1,15]*OM115*OM315+s.In[5,15]*OMp215-s.In[9,15]*OM115*OM315
    Cq315 = -s.trq[3,15]-s.In[1,15]*OM115*OM215+s.In[5,15]*OM115*OM215+s.In[9,15]*OMp315
    Fs114 = -s.frc[1,14]+s.m[14]*(ALPHA114+BETA314*s.l[3,14])
    Fs214 = -s.frc[2,14]+s.m[14]*(ALPHA214+BETA614*s.l[3,14])
    Fs314 = -s.frc[3,14]+s.m[14]*(ALPHA314+BS914*s.l[3,14])
    Fq114 = Fs114-s.frc[1,15]*C15-s.frc[3,15]*S15
    Fq214 = -s.frc[2,15]+Fs214
    Fq314 = Fs314+s.frc[1,15]*S15-s.frc[3,15]*C15
    Cq114 = -s.trq[1,14]+s.In[1,14]*OMp113-s.In[5,14]*OM213*OM313+s.In[9,14]*OM213*OM313+s.frc[2,15]*s.dpt[3,9]+Cq115*C15+Cq315*S15-Fs214*s.l[3,14]
    Cq214 = -s.trq[2,14]+Cq215+s.In[1,14]*OM113*OM313+s.In[5,14]*OMp213-s.In[9,14]*OM113*OM313+Fs114*s.l[3,14]+s.dpt[3,9]*(-s.frc[1,15]*C15-s.frc[3,15]*S15)
    Cq314 = -s.trq[3,14]-s.In[1,14]*OM113*OM213+s.In[5,14]*OM113*OM213+s.In[9,14]*OMp313-Cq115*S15+Cq315*C15
    Fs113 = -s.frc[1,13]+s.m[13]*(ALPHA113+BETA313*s.l[3,13])
    Fs213 = -s.frc[2,13]+s.m[13]*(ALPHA213+BETA613*s.l[3,13])
    Fs313 = -s.frc[3,13]+s.m[13]*(ALPHA312+BS913*s.l[3,13])
    Fq113 = Fq114+Fs113
    Fq213 = Fq214+Fs213
    Fq313 = Fq314+Fs313
    Cq113 = -s.trq[1,13]+Cq114+s.In[1,13]*OMp113-s.In[5,13]*OM213*OM313+s.In[9,13]*OM213*OM313-Fq214*Dz143-Fs213*s.l[3,13]
    Cq213 = -s.trq[2,13]+Cq214+s.In[1,13]*OM113*OM313+s.In[5,13]*OMp213-s.In[9,13]*OM113*OM313+Fq114*Dz143+Fs113*s.l[3,13]
    Cq313 = -s.trq[3,13]+Cq314-s.In[1,13]*OM113*OM213+s.In[5,13]*OM113*OM213+s.In[9,13]*OMp313
    Fq112 = Fq113*C13-Fq213*S13
    Fq212 = Fq113*S13+Fq213*C13
    Cq112 = Cq113*C13-Cq213*S13
    Cq212 = Cq113*S13+Cq213*C13
    Fs111 = -s.frc[1,11]+s.m[11]*(ALPHA111+BETA311*s.l[3,11])
    Fs211 = -s.frc[2,11]+s.m[11]*(ALPHA211+BETA611*s.l[3,11])
    Fs311 = -s.frc[3,11]+s.m[11]*(ALPHA311+BS911*s.l[3,11])
    Fq111 = Fs111+Fs116+Fq112*C12+Fq117*C17+Fq120*C20+Fq313*S12+Fq317*S17+Fq320*S20
    Fq211 = Fq212+Fq217+Fq220+Fs211+Fs216
    Fq311 = Fs311+Fs316-Fq112*S12-Fq117*S17-Fq120*S20+Fq313*C12+Fq317*C17+Fq320*C20
    Cq111 = -s.trq[1,11]+Cq116+s.In[1,11]*OMp111-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311+Cq112*C12+Cq117*C17+Cq120*C20+Cq313*S12+Cq317*S17+Cq320*S20-Fq217*s.dpt[3,6]-Fq220*s.dpt[3,7]-Fs211*s.l[3,11]+s.dpt[2,6]*(-Fq117*S17+Fq317*C17)+s.dpt[2,7]*(-Fq120*S20+Fq320*C20)
    Cq211 = -s.trq[2,11]+Cq212+Cq216+Cq217+Cq220-q[16]*Fs316+s.In[1,11]*OM111*OM311+s.In[5,11]*OMp211-s.In[9,11]*OM111*OM311+Fs111*s.l[3,11]+s.dpt[3,6]*(Fq117*C17+Fq317*S17)+s.dpt[3,7]*(Fq120*C20+Fq320*S20)
    Cq311 = -s.trq[3,11]+Cq316+q[16]*Fs216-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OMp311-Cq112*S12-Cq117*S17-Cq120*S20+Cq313*C12+Cq317*C17+Cq320*C20-s.dpt[2,6]*(Fq117*C17+Fq317*S17)-s.dpt[2,7]*(Fq120*C20+Fq320*S20)
    Fs110 = -s.frc[1,10]+s.m[10]*(ALPHA110+BS110*s.l[1,10])
    Fs210 = -s.frc[2,10]+s.m[10]*(ALPHA210+BETA410*s.l[1,10])
    Fs310 = -s.frc[3,10]+s.m[10]*(ALPHA310+BETA710*s.l[1,10])
    Fq110 = Fs110+Fq111*C11+Fq311*S11
    Fq210 = Fq211+Fs210
    Fq310 = Fs310-Fq111*S11+Fq311*C11
    Cq110 = -s.trq[1,10]+s.In[1,10]*OMp19-s.In[5,10]*OM29*OM39+s.In[9,10]*OM29*OM39+Cq111*C11+Cq311*S11
    Cq210 = -s.trq[2,10]+Cq211+s.In[1,10]*OM19*OM39+s.In[5,10]*OMp29-s.In[9,10]*OM19*OM39-Fs310*s.l[1,10]-s.dpt[1,5]*(-Fq111*S11+Fq311*C11)
    Cq310 = -s.trq[3,10]-s.In[1,10]*OM19*OM29+s.In[5,10]*OM19*OM29+s.In[9,10]*OMp39-Cq111*S11+Cq311*C11+Fq211*s.dpt[1,5]+Fs210*s.l[1,10]
    Fs19 = -s.frc[1,9]+s.m[9]*(ALPHA19+BS19*s.l[1,9])
    Fs29 = -s.frc[2,9]+s.m[9]*(ALPHA29+BETA49*s.l[1,9])
    Fs39 = -s.frc[3,9]+s.m[9]*(ALPHA39+BETA79*s.l[1,9])
    Fq19 = Fq110+Fs19
    Fq29 = Fq210+Fs29
    Fq39 = Fq310+Fs39
    Cq19 = -s.trq[1,9]+Cq110+s.In[1,9]*OMp19-s.In[5,9]*OM29*OM39+s.In[9,9]*OM29*OM39
    Cq29 = -s.trq[2,9]+Cq210+s.In[1,9]*OM19*OM39+s.In[5,9]*OMp29-s.In[9,9]*OM19*OM39-Fq310*Dz101-Fs39*s.l[1,9]
    Cq39 = -s.trq[3,9]+Cq310-s.In[1,9]*OM19*OM29+s.In[5,9]*OM19*OM29+s.In[9,9]*OMp39+Fq210*Dz101+Fs29*s.l[1,9]
    Fs18 = -s.frc[1,8]+s.m[8]*(ALPHA18+BS18*s.l[1,8])
    Fs28 = -s.frc[2,8]+s.m[8]*(ALPHA28+BETA48*s.l[1,8])
    Fs38 = -s.frc[3,8]+s.m[8]*(ALPHA38+BETA78*s.l[1,8])
    Fq18 = Fs18+Fq19*C9+Fq39*S9
    Fq28 = Fq29+Fs28
    Fq38 = Fs38-Fq19*S9+Fq39*C9
    Cq18 = -s.trq[1,8]+s.In[1,8]*OMp18-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38+Cq19*C9+Cq39*S9
    Cq28 = -s.trq[2,8]+Cq29+s.In[1,8]*OM18*OM38+s.In[5,8]*OMp28-s.In[9,8]*OM18*OM38-Fs38*s.l[1,8]-s.dpt[1,3]*(-Fq19*S9+Fq39*C9)
    Cq38 = -s.trq[3,8]-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OMp38-Cq19*S9+Cq39*C9+Fq29*s.dpt[1,3]+Fs28*s.l[1,8]
    Fs17 = -s.frc[1,7]+s.m[7]*(ALPHA17+BS17*s.l[1,7])
    Fs27 = -s.frc[2,7]+s.m[7]*(ALPHA27+BETA47*s.l[1,7])
    Fs37 = -s.frc[3,7]+s.m[7]*(ALPHA37+BETA77*s.l[1,7])
    Fq17 = Fs17+Fq18*C8+Fq38*S8
    Fq27 = Fq28+Fs27
    Fq37 = Fs37-Fq18*S8+Fq38*C8
    Cq17 = -s.trq[1,7]+s.In[1,7]*OMp17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37+Cq18*C8+Cq38*S8
    Cq27 = -s.trq[2,7]+Cq28+s.In[1,7]*OM17*OM37+s.In[5,7]*OMp27-s.In[9,7]*OM17*OM37-Fs37*s.l[1,7]-s.dpt[1,2]*(-Fq18*S8+Fq38*C8)
    Cq37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OMp37-Cq18*S8+Cq38*C8+Fq28*s.dpt[1,2]+Fs27*s.l[1,7]
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA16+BS16*s.l[1,6])
    Fs26 = -s.frc[2,6]+s.m[6]*(ALPHA25+BETA46*s.l[1,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA36+BETA76*s.l[1,6])
    Fq16 = Fs16+Fq17*C7+Fq37*S7
    Fq26 = Fq27+Fs26
    Fq36 = Fs36-Fq17*S7+Fq37*C7
    Cq16 = -s.trq[1,6]+s.In[1,6]*OMp16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+Cq17*C7+Cq37*S7
    Cq26 = -s.trq[2,6]+Cq27+s.In[1,6]*OM16*OM36+s.In[5,6]*OMp26-s.In[9,6]*OM16*OM36-Fs36*s.l[1,6]-s.dpt[1,1]*(-Fq17*S7+Fq37*C7)
    Cq36 = -s.trq[3,6]-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36-Cq17*S7+Cq37*C7+Fq27*s.dpt[1,1]+Fs26*s.l[1,6]
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
    Qq[10] = Fq110
    Qq[11] = Cq211
    Qq[12] = Cq212
    Qq[13] = Cq313
    Qq[14] = Fq314
    Qq[15] = Cq215
    Qq[16] = Fs116
    Qq[17] = Cq217
    Qq[18] = Cq218
    Qq[19] = Cq219
    Qq[20] = Cq220
    Qq[21] = Cq221
    Qq[22] = Cq222

# Number of continuation lines = 0


