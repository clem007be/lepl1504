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
#	==> Generation Date: Thu Mar 23 15:37:09 2023
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

 
# Backward Dynamics

    Fq121 = -s.frc[1,21]-s.frc[1,22]*C22-s.frc[3,22]*S22
    Fq221 = -s.frc[2,21]-s.frc[2,22]
    Fq321 = -s.frc[3,21]+s.frc[1,22]*S22-s.frc[3,22]*C22
    Cq121 = -s.trq[1,21]-s.trq[1,22]*C22-s.trq[3,22]*S22
    Cq221 = -s.trq[2,21]-s.trq[2,22]-s.dpt[1,17]*(s.frc[1,22]*S22-s.frc[3,22]*C22)
    Cq321 = -s.trq[3,21]-s.frc[2,22]*s.dpt[1,17]+s.trq[1,22]*S22-s.trq[3,22]*C22
    Fq120 = -s.frc[1,20]+Fq121*C21+Fq321*S21
    Fq220 = -s.frc[2,20]+Fq221
    Fq320 = -s.frc[3,20]-Fq121*S21+Fq321*C21
    Cq120 = -s.trq[1,20]+Cq121*C21+Cq321*S21-Fq221*s.dpt[3,15]
    Cq220 = -s.trq[2,20]+Cq221-s.dpt[1,15]*(-Fq121*S21+Fq321*C21)+s.dpt[3,15]*(Fq121*C21+Fq321*S21)
    Cq320 = -s.trq[3,20]-Cq121*S21+Cq321*C21+Fq221*s.dpt[1,15]
    Fq118 = -s.frc[1,18]-s.frc[1,19]*C19-s.frc[3,19]*S19
    Fq218 = -s.frc[2,18]-s.frc[2,19]
    Fq318 = -s.frc[3,18]+s.frc[1,19]*S19-s.frc[3,19]*C19
    Cq118 = -s.trq[1,18]-s.trq[1,19]*C19-s.trq[3,19]*S19
    Cq218 = -s.trq[2,18]-s.trq[2,19]-s.dpt[1,14]*(s.frc[1,19]*S19-s.frc[3,19]*C19)
    Cq318 = -s.trq[3,18]-s.frc[2,19]*s.dpt[1,14]+s.trq[1,19]*S19-s.trq[3,19]*C19
    Fq117 = -s.frc[1,17]+Fq118*C18+Fq318*S18
    Fq217 = -s.frc[2,17]+Fq218
    Fq317 = -s.frc[3,17]-Fq118*S18+Fq318*C18
    Cq117 = -s.trq[1,17]+Cq118*C18+Cq318*S18-Fq218*s.dpt[3,12]
    Cq217 = -s.trq[2,17]+Cq218-s.dpt[1,12]*(-Fq118*S18+Fq318*C18)+s.dpt[3,12]*(Fq118*C18+Fq318*S18)
    Cq317 = -s.trq[3,17]-Cq118*S18+Cq318*C18+Fq218*s.dpt[1,12]
    Fq114 = -s.frc[1,14]-s.frc[1,15]*C15-s.frc[3,15]*S15
    Fq214 = -s.frc[2,14]-s.frc[2,15]
    Fq314 = -s.frc[3,14]+s.frc[1,15]*S15-s.frc[3,15]*C15
    Cq114 = -s.trq[1,14]+s.frc[2,15]*s.dpt[3,9]-s.trq[1,15]*C15-s.trq[3,15]*S15
    Cq214 = -s.trq[2,14]-s.trq[2,15]+s.dpt[3,9]*(-s.frc[1,15]*C15-s.frc[3,15]*S15)
    Cq314 = -s.trq[3,14]+s.trq[1,15]*S15-s.trq[3,15]*C15
    Fq113 = -s.frc[1,13]+Fq114
    Fq213 = -s.frc[2,13]+Fq214
    Fq313 = -s.frc[3,13]+Fq314
    Cq113 = -s.trq[1,13]+Cq114-Fq214*Dz143
    Cq213 = -s.trq[2,13]+Cq214+Fq114*Dz143
    Cq313 = -s.trq[3,13]+Cq314
    Fq112 = Fq113*C13-Fq213*S13
    Fq212 = Fq113*S13+Fq213*C13
    Cq112 = Cq113*C13-Cq213*S13
    Cq212 = Cq113*S13+Cq213*C13
    Fq111 = -s.frc[1,11]-s.frc[1,16]+Fq112*C12+Fq117*C17+Fq120*C20+Fq313*S12+Fq317*S17+Fq320*S20
    Fq211 = -s.frc[2,11]-s.frc[2,16]+Fq212+Fq217+Fq220
    Fq311 = -s.frc[3,11]-s.frc[3,16]-Fq112*S12-Fq117*S17-Fq120*S20+Fq313*C12+Fq317*C17+Fq320*C20
    Cq111 = -s.trq[1,11]-s.trq[1,16]+Cq112*C12+Cq117*C17+Cq120*C20+Cq313*S12+Cq317*S17+Cq320*S20-Fq217*s.dpt[3,6]-Fq220*s.dpt[3,7]+s.dpt[2,6]*(-Fq117*S17+Fq317*C17)+s.dpt[2,7]*(-Fq120*S20+Fq320*C20)
    Cq211 = -s.trq[2,11]-s.trq[2,16]+Cq212+Cq217+Cq220+q[16]*s.frc[3,16]+s.dpt[3,6]*(Fq117*C17+Fq317*S17)+s.dpt[3,7]*(Fq120*C20+Fq320*S20)
    Cq311 = -s.trq[3,11]-s.trq[3,16]-q[16]*s.frc[2,16]-Cq112*S12-Cq117*S17-Cq120*S20+Cq313*C12+Cq317*C17+Cq320*C20-s.dpt[2,6]*(Fq117*C17+Fq317*S17)-s.dpt[2,7]*(Fq120*C20+Fq320*S20)
    Fq110 = -s.frc[1,10]+Fq111*C11+Fq311*S11
    Fq210 = -s.frc[2,10]+Fq211
    Fq310 = -s.frc[3,10]-Fq111*S11+Fq311*C11
    Cq110 = -s.trq[1,10]+Cq111*C11+Cq311*S11
    Cq210 = -s.trq[2,10]+Cq211-s.dpt[1,5]*(-Fq111*S11+Fq311*C11)
    Cq310 = -s.trq[3,10]-Cq111*S11+Cq311*C11+Fq211*s.dpt[1,5]
    Fq19 = -s.frc[1,9]+Fq110
    Fq29 = -s.frc[2,9]+Fq210
    Fq39 = -s.frc[3,9]+Fq310
    Cq19 = -s.trq[1,9]+Cq110
    Cq29 = -s.trq[2,9]+Cq210-Fq310*Dz101
    Cq39 = -s.trq[3,9]+Cq310+Fq210*Dz101
    Fq18 = -s.frc[1,8]+Fq19*C9+Fq39*S9
    Fq28 = -s.frc[2,8]+Fq29
    Fq38 = -s.frc[3,8]-Fq19*S9+Fq39*C9
    Cq18 = -s.trq[1,8]+Cq19*C9+Cq39*S9
    Cq28 = -s.trq[2,8]+Cq29-s.dpt[1,3]*(-Fq19*S9+Fq39*C9)
    Cq38 = -s.trq[3,8]-Cq19*S9+Cq39*C9+Fq29*s.dpt[1,3]
    Fq17 = -s.frc[1,7]+Fq18*C8+Fq38*S8
    Fq27 = -s.frc[2,7]+Fq28
    Fq37 = -s.frc[3,7]-Fq18*S8+Fq38*C8
    Cq17 = -s.trq[1,7]+Cq18*C8+Cq38*S8
    Cq27 = -s.trq[2,7]+Cq28-s.dpt[1,2]*(-Fq18*S8+Fq38*C8)
    Cq37 = -s.trq[3,7]-Cq18*S8+Cq38*C8+Fq28*s.dpt[1,2]
    Fq16 = -s.frc[1,6]+Fq17*C7+Fq37*S7
    Fq26 = -s.frc[2,6]+Fq27
    Fq36 = -s.frc[3,6]-Fq17*S7+Fq37*C7
    Cq16 = -s.trq[1,6]+Cq17*C7+Cq37*S7
    Cq26 = -s.trq[2,6]+Cq27-s.dpt[1,1]*(-Fq17*S7+Fq37*C7)
    Cq36 = -s.trq[3,6]-Cq17*S7+Cq37*C7+Fq27*s.dpt[1,1]
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
    Qq[15] = -s.trq[2,15]
    Qq[16] = -s.frc[1,16]
    Qq[17] = Cq217
    Qq[18] = Cq218
    Qq[19] = -s.trq[2,19]
    Qq[20] = Cq220
    Qq[21] = Cq221
    Qq[22] = -s.trq[2,22]

# Number of continuation lines = 0


