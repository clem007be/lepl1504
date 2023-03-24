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
#	==> Generation Date: Fri Mar 24 17:30:30 2023
#
#	==> Project name: Livrable2
#
#	==> Number of joints: 25
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
    S25 = sin(q[25])
    C25 = cos(q[25])
 
# Augmented Joint Position Vectors

    Dz243 = q[24]+s.dpt[3,16]
 
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
    ALPHA17 = C7*(ALPHA16+BS16*s.dpt[1,2])-S7*(ALPHA36+BETA76*s.dpt[1,2])
    ALPHA27 = ALPHA25+BETA46*s.dpt[1,2]
    ALPHA37 = C7*(ALPHA36+BETA76*s.dpt[1,2])+S7*(ALPHA16+BS16*s.dpt[1,2])
    OM18 = OM16*C8-OM36*S8
    OM28 = qd[8]+OM26
    OM38 = OM16*S8+OM36*C8
    OMp18 = C8*(OMp16-qd[8]*OM36)-S8*(OMp36+qd[8]*OM16)
    OMp28 = qdd[8]+OMp26
    OMp38 = C8*(OMp36+qd[8]*OM16)+S8*(OMp16-qd[8]*OM36)
    ALPHA18 = C8*(ALPHA16+BETA36*s.dpt[3,3]+BS16*s.dpt[1,3])-S8*(ALPHA36+BETA76*s.dpt[1,3]+BS96*s.dpt[3,3])
    ALPHA28 = ALPHA25+BETA46*s.dpt[1,3]+BETA66*s.dpt[3,3]
    ALPHA38 = C8*(ALPHA36+BETA76*s.dpt[1,3]+BS96*s.dpt[3,3])+S8*(ALPHA16+BETA36*s.dpt[3,3]+BS16*s.dpt[1,3])
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
    ALPHA110 = C10*(ALPHA19+BETA39*s.dpt[3,8])-S10*(ALPHA38+BS99*s.dpt[3,8])
    ALPHA210 = ALPHA29+BETA69*s.dpt[3,8]
    ALPHA310 = C10*(ALPHA38+BS99*s.dpt[3,8])+S10*(ALPHA19+BETA39*s.dpt[3,8])
    OM111 = OM16*C11+OM26*S11
    OM211 = -OM16*S11+OM26*C11
    OM311 = qd[11]+OM36
    OMp111 = C11*(OMp16+qd[11]*OM26)+S11*(OMp26-qd[11]*OM16)
    OMp211 = C11*(OMp26-qd[11]*OM16)-S11*(OMp16+qd[11]*OM26)
    OMp311 = qdd[11]+OMp36
    BS111 = -OM211*OM211-OM311*OM311
    BS211 = OM111*OM211
    BS311 = OM111*OM311
    BETA411 = BS211+OMp311
    BETA711 = BS311-OMp211
    ALPHA111 = C11*(ALPHA16+BETA36*s.dpt[3,5]+BS16*s.dpt[1,5])+S11*(ALPHA25+BETA46*s.dpt[1,5]+BETA66*s.dpt[3,5])
    ALPHA211 = C11*(ALPHA25+BETA46*s.dpt[1,5]+BETA66*s.dpt[3,5])-S11*(ALPHA16+BETA36*s.dpt[3,5]+BS16*s.dpt[1,5])
    ALPHA311 = ALPHA36+BETA76*s.dpt[1,5]+BS96*s.dpt[3,5]
    OM112 = OM111*C12-OM311*S12
    OM212 = qd[12]+OM211
    OM312 = OM111*S12+OM311*C12
    OMp112 = C12*(OMp111-qd[12]*OM311)-S12*(OMp311+qd[12]*OM111)
    OMp212 = qdd[12]+OMp211
    OMp312 = C12*(OMp311+qd[12]*OM111)+S12*(OMp111-qd[12]*OM311)
    BS112 = -OM212*OM212-OM312*OM312
    BS212 = OM112*OM212
    BS312 = OM112*OM312
    BETA412 = BS212+OMp312
    BETA712 = BS312-OMp212
    ALPHA112 = C12*(ALPHA111+BS111*s.dpt[1,11])-S12*(ALPHA311+BETA711*s.dpt[1,11])
    ALPHA212 = ALPHA211+BETA411*s.dpt[1,11]
    ALPHA312 = C12*(ALPHA311+BETA711*s.dpt[1,11])+S12*(ALPHA111+BS111*s.dpt[1,11])
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
    ALPHA113 = C13*(ALPHA112+BS112*s.dpt[1,12])-S13*(ALPHA312+BETA712*s.dpt[1,12])
    ALPHA213 = ALPHA212+BETA412*s.dpt[1,12]
    ALPHA313 = C13*(ALPHA312+BETA712*s.dpt[1,12])+S13*(ALPHA112+BS112*s.dpt[1,12])
    OM114 = OM113*C14-OM313*S14
    OM214 = qd[14]+OM213
    OM314 = OM113*S14+OM313*C14
    OMp114 = C14*(OMp113-qd[14]*OM313)-S14*(OMp313+qd[14]*OM113)
    OMp214 = qdd[14]+OMp213
    OMp314 = C14*(OMp313+qd[14]*OM113)+S14*(OMp113-qd[14]*OM313)
    BS114 = -OM214*OM214-OM314*OM314
    BS214 = OM114*OM214
    BS314 = OM114*OM314
    BETA414 = BS214+OMp314
    BETA714 = BS314-OMp214
    ALPHA114 = C14*(ALPHA113+BS113*s.dpt[1,13])-S14*(ALPHA313+BETA713*s.dpt[1,13])
    ALPHA214 = ALPHA213+BETA413*s.dpt[1,13]
    ALPHA314 = C14*(ALPHA313+BETA713*s.dpt[1,13])+S14*(ALPHA113+BS113*s.dpt[1,13])
    OM115 = OM114*C15-OM314*S15
    OM215 = qd[15]+OM214
    OM315 = OM114*S15+OM314*C15
    OMp115 = C15*(OMp114-qd[15]*OM314)-S15*(OMp314+qd[15]*OM114)
    OMp215 = qdd[15]+OMp214
    OMp315 = C15*(OMp314+qd[15]*OM114)+S15*(OMp114-qd[15]*OM314)
    BS315 = OM115*OM315
    BS615 = OM215*OM315
    BS915 = -OM115*OM115-OM215*OM215
    BETA315 = BS315+OMp215
    BETA615 = BS615-OMp115
    ALPHA115 = C15*(ALPHA114+BS114*s.dpt[1,14])-S15*(ALPHA314+BETA714*s.dpt[1,14])
    ALPHA215 = ALPHA214+BETA414*s.dpt[1,14]
    ALPHA315 = C15*(ALPHA314+BETA714*s.dpt[1,14])+S15*(ALPHA114+BS114*s.dpt[1,14])
    OM116 = OM115*C16+OM215*S16
    OM216 = -OM115*S16+OM215*C16
    OM316 = qd[16]+OM315
    OMp116 = C16*(OMp115+qd[16]*OM215)+S16*(OMp215-qd[16]*OM115)
    OMp216 = C16*(OMp215-qd[16]*OM115)-S16*(OMp115+qd[16]*OM215)
    OMp316 = qdd[16]+OMp315
    ALPHA116 = C16*(ALPHA115+BETA315*s.dpt[3,15])+S16*(ALPHA215+BETA615*s.dpt[3,15])
    ALPHA216 = C16*(ALPHA215+BETA615*s.dpt[3,15])-S16*(ALPHA115+BETA315*s.dpt[3,15])
    ALPHA316 = ALPHA315+BS915*s.dpt[3,15]
    OM117 = OM116*C17-OM316*S17
    OM217 = qd[17]+OM216
    OM317 = OM116*S17+OM316*C17
    OMp117 = C17*(OMp116-qd[17]*OM316)-S17*(OMp316+qd[17]*OM116)
    OMp217 = qdd[17]+OMp216
    OMp317 = C17*(OMp316+qd[17]*OM116)+S17*(OMp116-qd[17]*OM316)
    BS317 = OM117*OM317
    BS617 = OM217*OM317
    BS917 = -OM117*OM117-OM217*OM217
    BETA317 = BS317+OMp217
    BETA617 = BS617-OMp117
    ALPHA117 = ALPHA116*C17-ALPHA316*S17
    ALPHA317 = ALPHA116*S17+ALPHA316*C17
    OM118 = OM117*C18-OM317*S18
    OM218 = qd[18]+OM217
    OM318 = OM117*S18+OM317*C18
    OMp118 = C18*(OMp117-qd[18]*OM317)-S18*(OMp317+qd[18]*OM117)
    OMp218 = qdd[18]+OMp217
    OMp318 = C18*(OMp317+qd[18]*OM117)+S18*(OMp117-qd[18]*OM317)
    BS118 = -OM218*OM218-OM318*OM318
    BS218 = OM118*OM218
    BS318 = OM118*OM318
    BS518 = -OM118*OM118-OM318*OM318
    BS618 = OM218*OM318
    BS918 = -OM118*OM118-OM218*OM218
    BETA218 = BS218-OMp318
    BETA318 = BS318+OMp218
    BETA418 = BS218+OMp318
    BETA618 = BS618-OMp118
    BETA718 = BS318-OMp218
    BETA818 = BS618+OMp118
    ALPHA118 = C18*(ALPHA117+BETA317*s.dpt[3,17])-S18*(ALPHA317+BS917*s.dpt[3,17])
    ALPHA218 = ALPHA216+BETA617*s.dpt[3,17]
    ALPHA318 = C18*(ALPHA317+BS917*s.dpt[3,17])+S18*(ALPHA117+BETA317*s.dpt[3,17])
    OM119 = OM118*C19-OM318*S19
    OM219 = qd[19]+OM218
    OM319 = OM118*S19+OM318*C19
    OMp119 = C19*(OMp118-qd[19]*OM318)-S19*(OMp318+qd[19]*OM118)
    OMp219 = qdd[19]+OMp218
    OMp319 = C19*(OMp318+qd[19]*OM118)+S19*(OMp118-qd[19]*OM318)
    BS119 = -OM219*OM219-OM319*OM319
    BS219 = OM119*OM219
    BS319 = OM119*OM319
    BETA419 = BS219+OMp319
    BETA719 = BS319-OMp219
    ALPHA119 = C19*(ALPHA118+BETA218*s.dpt[2,18]+BETA318*s.dpt[3,18]+BS118*s.dpt[1,18])-S19*(ALPHA318+BETA718*s.dpt[1,18]+BETA818*s.dpt[2,18]+BS918*s.dpt[3,18])
    ALPHA219 = ALPHA218+BETA418*s.dpt[1,18]+BETA618*s.dpt[3,18]+BS518*s.dpt[2,18]
    ALPHA319 = C19*(ALPHA318+BETA718*s.dpt[1,18]+BETA818*s.dpt[2,18]+BS918*s.dpt[3,18])+S19*(ALPHA118+BETA218*s.dpt[2,18]+BETA318*s.dpt[3,18]+BS118*s.dpt[1,18])
    OM120 = OM119*C20-OM319*S20
    OM220 = qd[20]+OM219
    OM320 = OM119*S20+OM319*C20
    OMp120 = C20*(OMp119-qd[20]*OM319)-S20*(OMp319+qd[20]*OM119)
    OMp220 = qdd[20]+OMp219
    OMp320 = C20*(OMp319+qd[20]*OM119)+S20*(OMp119-qd[20]*OM319)
    ALPHA120 = C20*(ALPHA119+BS119*s.dpt[1,23])-S20*(ALPHA319+BETA719*s.dpt[1,23])
    ALPHA220 = ALPHA219+BETA419*s.dpt[1,23]
    ALPHA320 = C20*(ALPHA319+BETA719*s.dpt[1,23])+S20*(ALPHA119+BS119*s.dpt[1,23])
    OM121 = OM118*C21-OM318*S21
    OM221 = qd[21]+OM218
    OM321 = OM118*S21+OM318*C21
    OMp121 = C21*(OMp118-qd[21]*OM318)-S21*(OMp318+qd[21]*OM118)
    OMp221 = qdd[21]+OMp218
    OMp321 = C21*(OMp318+qd[21]*OM118)+S21*(OMp118-qd[21]*OM318)
    BS121 = -OM221*OM221-OM321*OM321
    BS221 = OM121*OM221
    BS321 = OM121*OM321
    BETA421 = BS221+OMp321
    BETA721 = BS321-OMp221
    ALPHA121 = C21*(ALPHA118+BETA218*s.dpt[2,19]+BETA318*s.dpt[3,19]+BS118*s.dpt[1,19])-S21*(ALPHA318+BETA718*s.dpt[1,19]+BETA818*s.dpt[2,19]+BS918*s.dpt[3,19])
    ALPHA221 = ALPHA218+BETA418*s.dpt[1,19]+BETA618*s.dpt[3,19]+BS518*s.dpt[2,19]
    ALPHA321 = C21*(ALPHA318+BETA718*s.dpt[1,19]+BETA818*s.dpt[2,19]+BS918*s.dpt[3,19])+S21*(ALPHA118+BETA218*s.dpt[2,19]+BETA318*s.dpt[3,19]+BS118*s.dpt[1,19])
    OM122 = OM121*C22-OM321*S22
    OM222 = qd[22]+OM221
    OM322 = OM121*S22+OM321*C22
    OMp122 = C22*(OMp121-qd[22]*OM321)-S22*(OMp321+qd[22]*OM121)
    OMp222 = qdd[22]+OMp221
    OMp322 = C22*(OMp321+qd[22]*OM121)+S22*(OMp121-qd[22]*OM321)
    ALPHA122 = C22*(ALPHA121+BS121*s.dpt[1,26])-S22*(ALPHA321+BETA721*s.dpt[1,26])
    ALPHA222 = ALPHA221+BETA421*s.dpt[1,26]
    ALPHA322 = C22*(ALPHA321+BETA721*s.dpt[1,26])+S22*(ALPHA121+BS121*s.dpt[1,26])
    BS323 = OM118*OM318
    BS623 = OM218*OM318
    BS923 = -OM118*OM118-OM218*OM218
    BETA323 = BS323+OMp218
    BETA623 = BS623-OMp118
    ALPHA123 = ALPHA118+q[23]*BETA318+(2.0)*qd[23]*OM218+BS118*s.dpt[1,22]
    ALPHA223 = ALPHA218+q[23]*BETA618-(2.0)*qd[23]*OM118+BETA418*s.dpt[1,22]
    ALPHA323 = qdd[23]+ALPHA318+q[23]*BS918+BETA718*s.dpt[1,22]
    BS324 = OM115*OM315
    BS624 = OM215*OM315
    BS924 = -OM115*OM115-OM215*OM215
    BETA324 = BS324+OMp215
    BETA624 = BS624-OMp115
    ALPHA124 = ALPHA115+(2.0)*qd[24]*OM215+BETA315*Dz243
    ALPHA224 = ALPHA215-(2.0)*qd[24]*OM115+BETA615*Dz243
    ALPHA324 = qdd[24]+ALPHA315+BS915*Dz243
    OM125 = OM115*C25-OM315*S25
    OM225 = qd[25]+OM215
    OM325 = OM115*S25+OM315*C25
    OMp125 = C25*(OMp115-qd[25]*OM315)-S25*(OMp315+qd[25]*OM115)
    OMp225 = qdd[25]+OMp215
    OMp325 = C25*(OMp315+qd[25]*OM115)+S25*(OMp115-qd[25]*OM315)
    ALPHA125 = C25*(ALPHA124+BETA324*s.dpt[3,29])-S25*(ALPHA324+BS924*s.dpt[3,29])
    ALPHA225 = ALPHA224+BETA624*s.dpt[3,29]
    ALPHA325 = C25*(ALPHA324+BS924*s.dpt[3,29])+S25*(ALPHA124+BETA324*s.dpt[3,29])
 
# Backward Dynamics

    Fs125 = -s.frc[1,25]+s.m[25]*ALPHA125
    Fs225 = -s.frc[2,25]+s.m[25]*ALPHA225
    Fs325 = -s.frc[3,25]+s.m[25]*ALPHA325
    Cq125 = -s.trq[1,25]+s.In[1,25]*OMp125-s.In[5,25]*OM225*OM325+s.In[9,25]*OM225*OM325
    Cq225 = -s.trq[2,25]+s.In[1,25]*OM125*OM325+s.In[5,25]*OMp225-s.In[9,25]*OM125*OM325
    Cq325 = -s.trq[3,25]-s.In[1,25]*OM125*OM225+s.In[5,25]*OM125*OM225+s.In[9,25]*OMp325
    Fs124 = -s.frc[1,24]+s.m[24]*(ALPHA124+BETA324*s.l[3,24])
    Fs224 = -s.frc[2,24]+s.m[24]*(ALPHA224+BETA624*s.l[3,24])
    Fs324 = -s.frc[3,24]+s.m[24]*(ALPHA324+BS924*s.l[3,24])
    Fq124 = Fs124+Fs125*C25+Fs325*S25
    Fq224 = Fs224+Fs225
    Fq324 = Fs324-Fs125*S25+Fs325*C25
    Cq124 = -s.trq[1,24]+s.In[1,24]*OMp115-s.In[5,24]*OM215*OM315+s.In[9,24]*OM215*OM315+Cq125*C25+Cq325*S25-Fs224*s.l[3,24]-Fs225*s.dpt[3,29]
    Cq224 = -s.trq[2,24]+Cq225+s.In[1,24]*OM115*OM315+s.In[5,24]*OMp215-s.In[9,24]*OM115*OM315+Fs124*s.l[3,24]+s.dpt[3,29]*(Fs125*C25+Fs325*S25)
    Cq324 = -s.trq[3,24]-s.In[1,24]*OM115*OM215+s.In[5,24]*OM115*OM215+s.In[9,24]*OMp315-Cq125*S25+Cq325*C25
    Fs123 = -s.frc[1,23]+s.m[23]*(ALPHA123+BETA323*s.l[3,23])
    Fs223 = -s.frc[2,23]+s.m[23]*(ALPHA223+BETA623*s.l[3,23])
    Fs323 = -s.frc[3,23]+s.m[23]*(ALPHA323+BS923*s.l[3,23])
    Cq123 = -s.trq[1,23]+s.In[1,23]*OMp118-s.In[5,23]*OM218*OM318+s.In[9,23]*OM218*OM318-Fs223*s.l[3,23]
    Cq223 = -s.trq[2,23]+s.In[1,23]*OM118*OM318+s.In[5,23]*OMp218-s.In[9,23]*OM118*OM318+Fs123*s.l[3,23]
    Cq323 = -s.trq[3,23]-s.In[1,23]*OM118*OM218+s.In[5,23]*OM118*OM218+s.In[9,23]*OMp318
    Fs122 = -s.frc[1,22]+s.m[22]*ALPHA122
    Fs222 = -s.frc[2,22]+s.m[22]*ALPHA222
    Fs322 = -s.frc[3,22]+s.m[22]*ALPHA322
    Cq122 = -s.trq[1,22]+s.In[1,22]*OMp122-s.In[5,22]*OM222*OM322+s.In[9,22]*OM222*OM322
    Cq222 = -s.trq[2,22]+s.In[1,22]*OM122*OM322+s.In[5,22]*OMp222-s.In[9,22]*OM122*OM322
    Cq322 = -s.trq[3,22]-s.In[1,22]*OM122*OM222+s.In[5,22]*OM122*OM222+s.In[9,22]*OMp322
    Fs121 = -s.frc[1,21]+s.m[21]*(ALPHA121+BS121*s.l[1,21])
    Fs221 = -s.frc[2,21]+s.m[21]*(ALPHA221+BETA421*s.l[1,21])
    Fs321 = -s.frc[3,21]+s.m[21]*(ALPHA321+BETA721*s.l[1,21])
    Fq121 = Fs121+Fs122*C22+Fs322*S22
    Fq221 = Fs221+Fs222
    Fq321 = Fs321-Fs122*S22+Fs322*C22
    Cq121 = -s.trq[1,21]+s.In[1,21]*OMp121-s.In[5,21]*OM221*OM321+s.In[9,21]*OM221*OM321+Cq122*C22+Cq322*S22
    Cq221 = -s.trq[2,21]+Cq222+s.In[1,21]*OM121*OM321+s.In[5,21]*OMp221-s.In[9,21]*OM121*OM321-Fs321*s.l[1,21]-s.dpt[1,26]*(-Fs122*S22+Fs322*C22)
    Cq321 = -s.trq[3,21]-s.In[1,21]*OM121*OM221+s.In[5,21]*OM121*OM221+s.In[9,21]*OMp321-Cq122*S22+Cq322*C22+Fs221*s.l[1,21]+Fs222*s.dpt[1,26]
    Fs120 = -s.frc[1,20]+s.m[20]*ALPHA120
    Fs220 = -s.frc[2,20]+s.m[20]*ALPHA220
    Fs320 = -s.frc[3,20]+s.m[20]*ALPHA320
    Cq120 = -s.trq[1,20]+s.In[1,20]*OMp120-s.In[5,20]*OM220*OM320+s.In[9,20]*OM220*OM320
    Cq220 = -s.trq[2,20]+s.In[1,20]*OM120*OM320+s.In[5,20]*OMp220-s.In[9,20]*OM120*OM320
    Cq320 = -s.trq[3,20]-s.In[1,20]*OM120*OM220+s.In[5,20]*OM120*OM220+s.In[9,20]*OMp320
    Fs119 = -s.frc[1,19]+s.m[19]*(ALPHA119+BS119*s.l[1,19])
    Fs219 = -s.frc[2,19]+s.m[19]*(ALPHA219+BETA419*s.l[1,19])
    Fs319 = -s.frc[3,19]+s.m[19]*(ALPHA319+BETA719*s.l[1,19])
    Fq119 = Fs119+Fs120*C20+Fs320*S20
    Fq219 = Fs219+Fs220
    Fq319 = Fs319-Fs120*S20+Fs320*C20
    Cq119 = -s.trq[1,19]+s.In[1,19]*OMp119-s.In[5,19]*OM219*OM319+s.In[9,19]*OM219*OM319+Cq120*C20+Cq320*S20
    Cq219 = -s.trq[2,19]+Cq220+s.In[1,19]*OM119*OM319+s.In[5,19]*OMp219-s.In[9,19]*OM119*OM319-Fs319*s.l[1,19]-s.dpt[1,23]*(-Fs120*S20+Fs320*C20)
    Cq319 = -s.trq[3,19]-s.In[1,19]*OM119*OM219+s.In[5,19]*OM119*OM219+s.In[9,19]*OMp319-Cq120*S20+Cq320*C20+Fs219*s.l[1,19]+Fs220*s.dpt[1,23]
    Fs118 = -s.frc[1,18]+s.m[18]*(ALPHA118+BS118*s.l[1,18])
    Fs218 = -s.frc[2,18]+s.m[18]*(ALPHA218+BETA418*s.l[1,18])
    Fs318 = -s.frc[3,18]+s.m[18]*(ALPHA318+BETA718*s.l[1,18])
    Fq118 = Fs118+Fs123+Fq119*C19+Fq121*C21+Fq319*S19+Fq321*S21
    Fq218 = Fq219+Fq221+Fs218+Fs223
    Fq318 = Fs318+Fs323-Fq119*S19-Fq121*S21+Fq319*C19+Fq321*C21
    Cq118 = -s.trq[1,18]+Cq123-q[23]*Fs223+s.In[1,18]*OMp118-s.In[5,18]*OM218*OM318+s.In[9,18]*OM218*OM318+Cq119*C19+Cq121*C21+Cq319*S19+Cq321*S21-Fq219*s.dpt[3,18]-Fq221*s.dpt[3,19]+s.dpt[2,18]*(-Fq119*S19+Fq319*C19)+s.dpt[2,19]*(-Fq121*S21+Fq321*C21)
    Cq218 = -s.trq[2,18]+Cq219+Cq221+Cq223+q[23]*Fs123+s.In[1,18]*OM118*OM318+s.In[5,18]*OMp218-s.In[9,18]*OM118*OM318-Fs318*s.l[1,18]-Fs323*s.dpt[1,22]-s.dpt[1,18]*(-Fq119*S19+Fq319*C19)-s.dpt[1,19]*(-Fq121*S21+Fq321*C21)+s.dpt[3,18]*(Fq119*C19+Fq319*S19)+s.dpt[3,19]*(Fq121*C21+Fq321*S21)
    Cq318 = -s.trq[3,18]+Cq323-s.In[1,18]*OM118*OM218+s.In[5,18]*OM118*OM218+s.In[9,18]*OMp318-Cq119*S19-Cq121*S21+Cq319*C19+Cq321*C21+Fq219*s.dpt[1,18]+Fq221*s.dpt[1,19]+Fs218*s.l[1,18]+Fs223*s.dpt[1,22]-s.dpt[2,18]*(Fq119*C19+Fq319*S19)-s.dpt[2,19]*(Fq121*C21+Fq321*S21)
    Fs117 = -s.frc[1,17]+s.m[17]*(ALPHA117+BETA317*s.l[3,17])
    Fs217 = -s.frc[2,17]+s.m[17]*(ALPHA216+BETA617*s.l[3,17])
    Fs317 = -s.frc[3,17]+s.m[17]*(ALPHA317+BS917*s.l[3,17])
    Fq117 = Fs117+Fq118*C18+Fq318*S18
    Fq217 = Fq218+Fs217
    Fq317 = Fs317-Fq118*S18+Fq318*C18
    Cq117 = -s.trq[1,17]+s.In[1,17]*OMp117-s.In[5,17]*OM217*OM317+s.In[9,17]*OM217*OM317+Cq118*C18+Cq318*S18-Fq218*s.dpt[3,17]-Fs217*s.l[3,17]
    Cq217 = -s.trq[2,17]+Cq218+s.In[1,17]*OM117*OM317+s.In[5,17]*OMp217-s.In[9,17]*OM117*OM317+Fs117*s.l[3,17]+s.dpt[3,17]*(Fq118*C18+Fq318*S18)
    Cq317 = -s.trq[3,17]-s.In[1,17]*OM117*OM217+s.In[5,17]*OM117*OM217+s.In[9,17]*OMp317-Cq118*S18+Cq318*C18
    Fq116 = Fq117*C17+Fq317*S17
    Fq316 = -Fq117*S17+Fq317*C17
    Cq116 = Cq117*C17+Cq317*S17
    Cq316 = -Cq117*S17+Cq317*C17
    Fs115 = -s.frc[1,15]+s.m[15]*(ALPHA115+BETA315*s.l[3,15])
    Fs215 = -s.frc[2,15]+s.m[15]*(ALPHA215+BETA615*s.l[3,15])
    Fs315 = -s.frc[3,15]+s.m[15]*(ALPHA315+BS915*s.l[3,15])
    Fq115 = Fq124+Fs115+Fq116*C16-Fq217*S16
    Fq215 = Fq224+Fs215+Fq116*S16+Fq217*C16
    Fq315 = Fq316+Fq324+Fs315
    Cq115 = -s.trq[1,15]+Cq124+s.In[1,15]*OMp115-s.In[5,15]*OM215*OM315+s.In[9,15]*OM215*OM315+Cq116*C16-Cq217*S16-Fq224*Dz243-Fs215*s.l[3,15]-s.dpt[3,15]*(Fq116*S16+Fq217*C16)
    Cq215 = -s.trq[2,15]+Cq224+s.In[1,15]*OM115*OM315+s.In[5,15]*OMp215-s.In[9,15]*OM115*OM315+Cq116*S16+Cq217*C16+Fq124*Dz243+Fs115*s.l[3,15]+s.dpt[3,15]*(Fq116*C16-Fq217*S16)
    Cq315 = -s.trq[3,15]+Cq316+Cq324-s.In[1,15]*OM115*OM215+s.In[5,15]*OM115*OM215+s.In[9,15]*OMp315
    Fs114 = -s.frc[1,14]+s.m[14]*(ALPHA114+BS114*s.l[1,14])
    Fs214 = -s.frc[2,14]+s.m[14]*(ALPHA214+BETA414*s.l[1,14])
    Fs314 = -s.frc[3,14]+s.m[14]*(ALPHA314+BETA714*s.l[1,14])
    Fq114 = Fs114+Fq115*C15+Fq315*S15
    Fq214 = Fq215+Fs214
    Fq314 = Fs314-Fq115*S15+Fq315*C15
    Cq114 = -s.trq[1,14]+s.In[1,14]*OMp114-s.In[5,14]*OM214*OM314+s.In[9,14]*OM214*OM314+Cq115*C15+Cq315*S15
    Cq214 = -s.trq[2,14]+Cq215+s.In[1,14]*OM114*OM314+s.In[5,14]*OMp214-s.In[9,14]*OM114*OM314-Fs314*s.l[1,14]-s.dpt[1,14]*(-Fq115*S15+Fq315*C15)
    Cq314 = -s.trq[3,14]-s.In[1,14]*OM114*OM214+s.In[5,14]*OM114*OM214+s.In[9,14]*OMp314-Cq115*S15+Cq315*C15+Fq215*s.dpt[1,14]+Fs214*s.l[1,14]
    Fs113 = -s.frc[1,13]+s.m[13]*(ALPHA113+BS113*s.l[1,13])
    Fs213 = -s.frc[2,13]+s.m[13]*(ALPHA213+BETA413*s.l[1,13])
    Fs313 = -s.frc[3,13]+s.m[13]*(ALPHA313+BETA713*s.l[1,13])
    Fq113 = Fs113+Fq114*C14+Fq314*S14
    Fq213 = Fq214+Fs213
    Fq313 = Fs313-Fq114*S14+Fq314*C14
    Cq113 = -s.trq[1,13]+s.In[1,13]*OMp113-s.In[5,13]*OM213*OM313+s.In[9,13]*OM213*OM313+Cq114*C14+Cq314*S14
    Cq213 = -s.trq[2,13]+Cq214+s.In[1,13]*OM113*OM313+s.In[5,13]*OMp213-s.In[9,13]*OM113*OM313-Fs313*s.l[1,13]-s.dpt[1,13]*(-Fq114*S14+Fq314*C14)
    Cq313 = -s.trq[3,13]-s.In[1,13]*OM113*OM213+s.In[5,13]*OM113*OM213+s.In[9,13]*OMp313-Cq114*S14+Cq314*C14+Fq214*s.dpt[1,13]+Fs213*s.l[1,13]
    Fs112 = -s.frc[1,12]+s.m[12]*(ALPHA112+BS112*s.l[1,12])
    Fs212 = -s.frc[2,12]+s.m[12]*(ALPHA212+BETA412*s.l[1,12])
    Fs312 = -s.frc[3,12]+s.m[12]*(ALPHA312+BETA712*s.l[1,12])
    Fq112 = Fs112+Fq113*C13+Fq313*S13
    Fq212 = Fq213+Fs212
    Fq312 = Fs312-Fq113*S13+Fq313*C13
    Cq112 = -s.trq[1,12]+s.In[1,12]*OMp112-s.In[5,12]*OM212*OM312+s.In[9,12]*OM212*OM312+Cq113*C13+Cq313*S13
    Cq212 = -s.trq[2,12]+Cq213+s.In[1,12]*OM112*OM312+s.In[5,12]*OMp212-s.In[9,12]*OM112*OM312-Fs312*s.l[1,12]-s.dpt[1,12]*(-Fq113*S13+Fq313*C13)
    Cq312 = -s.trq[3,12]-s.In[1,12]*OM112*OM212+s.In[5,12]*OM112*OM212+s.In[9,12]*OMp312-Cq113*S13+Cq313*C13+Fq213*s.dpt[1,12]+Fs212*s.l[1,12]
    Fs111 = -s.frc[1,11]+s.m[11]*(ALPHA111+BS111*s.l[1,11])
    Fs211 = -s.frc[2,11]+s.m[11]*(ALPHA211+BETA411*s.l[1,11])
    Fs311 = -s.frc[3,11]+s.m[11]*(ALPHA311+BETA711*s.l[1,11])
    Fq111 = Fs111+Fq112*C12+Fq312*S12
    Fq211 = Fq212+Fs211
    Fq311 = Fs311-Fq112*S12+Fq312*C12
    Cq111 = -s.trq[1,11]+s.In[1,11]*OMp111-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311+Cq112*C12+Cq312*S12
    Cq211 = -s.trq[2,11]+Cq212+s.In[1,11]*OM111*OM311+s.In[5,11]*OMp211-s.In[9,11]*OM111*OM311-Fs311*s.l[1,11]-s.dpt[1,11]*(-Fq112*S12+Fq312*C12)
    Cq311 = -s.trq[3,11]-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OMp311-Cq112*S12+Cq312*C12+Fq212*s.dpt[1,11]+Fs211*s.l[1,11]
    Fs110 = -s.frc[1,10]+s.m[10]*ALPHA110
    Fs210 = -s.frc[2,10]+s.m[10]*ALPHA210
    Fs310 = -s.frc[3,10]+s.m[10]*ALPHA310
    Cq110 = -s.trq[1,10]+s.In[1,10]*OMp110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310
    Cq210 = -s.trq[2,10]+s.In[1,10]*OM110*OM310+s.In[5,10]*OMp210-s.In[9,10]*OM110*OM310
    Cq310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OMp310
    Fs19 = -s.frc[1,9]+s.m[9]*(ALPHA19+BETA39*s.l[3,9])
    Fs29 = -s.frc[2,9]+s.m[9]*(ALPHA29+BETA69*s.l[3,9])
    Fs39 = -s.frc[3,9]+s.m[9]*(ALPHA38+BS99*s.l[3,9])
    Fq19 = Fs19+Fs110*C10+Fs310*S10
    Fq29 = Fs210+Fs29
    Fq39 = Fs39-Fs110*S10+Fs310*C10
    Cq19 = -s.trq[1,9]+Cq110*C10+Cq310*S10-Fs210*s.dpt[3,8]-Fs29*s.l[3,9]
    Cq29 = -s.trq[2,9]+Cq210+Fs19*s.l[3,9]+s.dpt[3,8]*(Fs110*C10+Fs310*S10)
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
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA16+BETA36*s.l[3,6]+BS16*s.l[1,6])
    Fs26 = -s.frc[2,6]+s.m[6]*(ALPHA25+BETA46*s.l[1,6]+BETA66*s.l[3,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA36+BETA76*s.l[1,6]+BS96*s.l[3,6])
    Fq16 = Fs16+Fq111*C11+Fq18*C8-Fq211*S11+Fq39*S8+Fs17*C7+Fs37*S7
    Fq26 = Fq28+Fs26+Fs27+Fq111*S11+Fq211*C11
    Fq36 = Fq311+Fs36-Fq18*S8+Fq39*C8-Fs17*S7+Fs37*C7
    Cq16 = -s.trq[1,6]+s.In[1,6]*OMp16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+Cq111*C11+Cq17*C7+Cq18*C8-Cq211*S11+Cq37*S7+Cq39*S8-Fq28*s.dpt[3,3]-Fs26*s.l[3,6]-s.dpt[3,5]*(Fq111*S11+Fq211*C11)
    Cq26 = -s.trq[2,6]+Cq27+Cq28+s.In[1,6]*OM16*OM36+s.In[5,6]*OMp26-s.In[9,6]*OM16*OM36+Cq111*S11+Cq211*C11-Fq311*s.dpt[1,5]+Fs16*s.l[3,6]-Fs36*s.l[1,6]-s.dpt[1,2]*(-Fs17*S7+Fs37*C7)-s.dpt[1,3]*(-Fq18*S8+Fq39*C8)+s.dpt[3,3]*(Fq18*C8+Fq39*S8)+s.dpt[3,5]*(Fq111*C11-Fq211*S11)
    Cq36 = -s.trq[3,6]+Cq311-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36-Cq17*S7-Cq18*S8+Cq37*C7+Cq39*C8+Fq28*s.dpt[1,3]+Fs26*s.l[1,6]+Fs27*s.dpt[1,2]+s.dpt[1,5]*(Fq111*S11+Fq211*C11)
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
    Qq[11] = Cq311
    Qq[12] = Cq212
    Qq[13] = Cq213
    Qq[14] = Cq214
    Qq[15] = Cq215
    Qq[16] = Cq316
    Qq[17] = Cq217
    Qq[18] = Cq218
    Qq[19] = Cq219
    Qq[20] = Cq220
    Qq[21] = Cq221
    Qq[22] = Cq222
    Qq[23] = Fs323
    Qq[24] = Fq324
    Qq[25] = Cq225

# Number of continuation lines = 0


