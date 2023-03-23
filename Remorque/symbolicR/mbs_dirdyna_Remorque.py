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
#	==> Function: F1 - Recursive Direct Dynamics of tree-like MBS
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def dirdyna(M, c, s, tsim):
    q = s.q
    qd = s.qd
 
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
    C6p7 = C6*C7-S6*S7
    S6p7 = C6*S7+S6*C7
    C6p7p8 = C6p7*C8-S6p7*S8
    S6p7p8 = C6p7*S8+S6p7*C8
    C6p7p8p9 = C6p7p8*C9-S6p7p8*S9
    S6p7p8p9 = C6p7p8*S9+S6p7p8*C9
    C6p7p8p9p11 = C6p7p8p9*C11-S6p7p8p9*S11
    S6p7p8p9p11 = C6p7p8p9*S11+S6p7p8p9*C11
    C6p7p8p9p11p12 = C6p7p8p9p11*C12-S6p7p8p9p11*S12
    S6p7p8p9p11p12 = C6p7p8p9p11*S12+S6p7p8p9p11*C12
    C6p7p8p9p11p17 = C6p7p8p9p11*C17-S6p7p8p9p11*S17
    S6p7p8p9p11p17 = C6p7p8p9p11*S17+S6p7p8p9p11*C17
    C6p7p8p9p11p17p18 = C6p7p8p9p11p17*C18-S6p7p8p9p11p17*S18
    S6p7p8p9p11p17p18 = C6p7p8p9p11p17*S18+S6p7p8p9p11p17*C18
    C6p7p8p9p11p17p18p19 = C6p7p8p9p11p17p18*C19-S6p7p8p9p11p17p18*S19
    S6p7p8p9p11p17p18p19 = C6p7p8p9p11p17p18*S19+S6p7p8p9p11p17p18*C19
    C6p7p8p9p11p20 = C6p7p8p9p11*C20-S6p7p8p9p11*S20
    S6p7p8p9p11p20 = C6p7p8p9p11*S20+S6p7p8p9p11*C20
    C6p7p8p9p11p20p21 = C6p7p8p9p11p20*C21-S6p7p8p9p11p20*S21
    S6p7p8p9p11p20p21 = C6p7p8p9p11p20*S21+S6p7p8p9p11p20*C21
    C6p7p8p9p11p20p21p22 = C6p7p8p9p11p20p21*C22-S6p7p8p9p11p20p21*S22
    S6p7p8p9p11p20p21p22 = C6p7p8p9p11p20p21*S22+S6p7p8p9p11p20p21*C22
 
# Augmented Joint Position Vectors

    Dz101 = q[10]+s.dpt[1,4]
    Dz143 = q[14]+s.dpt[3,8]
 
# Forward Kinematics

 
# Backward Dynamics

    FF121 = -s.frc[1,21]-s.frc[1,22]*C22-s.frc[3,22]*S22
    FF221 = -s.frc[2,21]-s.frc[2,22]
    FF321 = -s.frc[3,21]+s.frc[1,22]*S22-s.frc[3,22]*C22
    CF121 = -s.trq[1,21]-s.trq[1,22]*C22-s.trq[3,22]*S22
    CF221 = -s.trq[2,21]-s.trq[2,22]-s.dpt[1,17]*(s.frc[1,22]*S22-s.frc[3,22]*C22)
    CF321 = -s.trq[3,21]-s.frc[2,22]*s.dpt[1,17]+s.trq[1,22]*S22-s.trq[3,22]*C22
    FF120 = -s.frc[1,20]+FF121*C21+FF321*S21
    FF220 = -s.frc[2,20]+FF221
    FF320 = -s.frc[3,20]-FF121*S21+FF321*C21
    CF120 = -s.trq[1,20]+CF121*C21+CF321*S21-FF221*s.dpt[3,15]
    CF220 = -s.trq[2,20]+CF221-s.dpt[1,15]*(-FF121*S21+FF321*C21)+s.dpt[3,15]*(FF121*C21+FF321*S21)
    CF320 = -s.trq[3,20]-CF121*S21+CF321*C21+FF221*s.dpt[1,15]
    FF118 = -s.frc[1,18]-s.frc[1,19]*C19-s.frc[3,19]*S19
    FF218 = -s.frc[2,18]-s.frc[2,19]
    FF318 = -s.frc[3,18]+s.frc[1,19]*S19-s.frc[3,19]*C19
    CF118 = -s.trq[1,18]-s.trq[1,19]*C19-s.trq[3,19]*S19
    CF218 = -s.trq[2,18]-s.trq[2,19]-s.dpt[1,14]*(s.frc[1,19]*S19-s.frc[3,19]*C19)
    CF318 = -s.trq[3,18]-s.frc[2,19]*s.dpt[1,14]+s.trq[1,19]*S19-s.trq[3,19]*C19
    FF117 = -s.frc[1,17]+FF118*C18+FF318*S18
    FF217 = -s.frc[2,17]+FF218
    FF317 = -s.frc[3,17]-FF118*S18+FF318*C18
    CF117 = -s.trq[1,17]+CF118*C18+CF318*S18-FF218*s.dpt[3,12]
    CF217 = -s.trq[2,17]+CF218-s.dpt[1,12]*(-FF118*S18+FF318*C18)+s.dpt[3,12]*(FF118*C18+FF318*S18)
    CF317 = -s.trq[3,17]-CF118*S18+CF318*C18+FF218*s.dpt[1,12]
    FF114 = -s.frc[1,14]-s.frc[1,15]*C15-s.frc[3,15]*S15
    FF214 = -s.frc[2,14]-s.frc[2,15]
    FF314 = -s.frc[3,14]+s.frc[1,15]*S15-s.frc[3,15]*C15
    CF114 = -s.trq[1,14]+s.frc[2,15]*s.dpt[3,9]-s.trq[1,15]*C15-s.trq[3,15]*S15
    CF214 = -s.trq[2,14]-s.trq[2,15]+s.dpt[3,9]*(-s.frc[1,15]*C15-s.frc[3,15]*S15)
    CF314 = -s.trq[3,14]+s.trq[1,15]*S15-s.trq[3,15]*C15
    FF113 = -s.frc[1,13]+FF114
    FF213 = -s.frc[2,13]+FF214
    FF313 = -s.frc[3,13]+FF314
    CF113 = -s.trq[1,13]+CF114-FF214*Dz143
    CF213 = -s.trq[2,13]+CF214+FF114*Dz143
    CF313 = -s.trq[3,13]+CF314
    FF112 = FF113*C13-FF213*S13
    FF212 = FF113*S13+FF213*C13
    CF112 = CF113*C13-CF213*S13
    CF212 = CF113*S13+CF213*C13
    FF111 = -s.frc[1,11]-s.frc[1,16]+FF112*C12+FF117*C17+FF120*C20+FF313*S12+FF317*S17+FF320*S20
    FF211 = -s.frc[2,11]-s.frc[2,16]+FF212+FF217+FF220
    FF311 = -s.frc[3,11]-s.frc[3,16]-FF112*S12-FF117*S17-FF120*S20+FF313*C12+FF317*C17+FF320*C20
    CF111 = -s.trq[1,11]-s.trq[1,16]+CF112*C12+CF117*C17+CF120*C20+CF313*S12+CF317*S17+CF320*S20-FF217*s.dpt[3,6]-FF220*s.dpt[3,7]+s.dpt[2,6]*(-FF117*S17+FF317*C17)+s.dpt[2,7]*(-FF120*S20+FF320*C20)
    CF211 = -s.trq[2,11]-s.trq[2,16]+CF212+CF217+CF220+q[16]*s.frc[3,16]+s.dpt[3,6]*(FF117*C17+FF317*S17)+s.dpt[3,7]*(FF120*C20+FF320*S20)
    CF311 = -s.trq[3,11]-s.trq[3,16]-q[16]*s.frc[2,16]-CF112*S12-CF117*S17-CF120*S20+CF313*C12+CF317*C17+CF320*C20-s.dpt[2,6]*(FF117*C17+FF317*S17)-s.dpt[2,7]*(FF120*C20+FF320*S20)
    FF110 = -s.frc[1,10]+FF111*C11+FF311*S11
    FF210 = -s.frc[2,10]+FF211
    FF310 = -s.frc[3,10]-FF111*S11+FF311*C11
    CF110 = -s.trq[1,10]+CF111*C11+CF311*S11
    CF210 = -s.trq[2,10]+CF211-s.dpt[1,5]*(-FF111*S11+FF311*C11)
    CF310 = -s.trq[3,10]-CF111*S11+CF311*C11+FF211*s.dpt[1,5]
    FF19 = -s.frc[1,9]+FF110
    FF29 = -s.frc[2,9]+FF210
    FF39 = -s.frc[3,9]+FF310
    CF19 = -s.trq[1,9]+CF110
    CF29 = -s.trq[2,9]+CF210-FF310*Dz101
    CF39 = -s.trq[3,9]+CF310+FF210*Dz101
    FF18 = -s.frc[1,8]+FF19*C9+FF39*S9
    FF28 = -s.frc[2,8]+FF29
    FF38 = -s.frc[3,8]-FF19*S9+FF39*C9
    CF18 = -s.trq[1,8]+CF19*C9+CF39*S9
    CF28 = -s.trq[2,8]+CF29-s.dpt[1,3]*(-FF19*S9+FF39*C9)
    CF38 = -s.trq[3,8]-CF19*S9+CF39*C9+FF29*s.dpt[1,3]
    FF17 = -s.frc[1,7]+FF18*C8+FF38*S8
    FF27 = -s.frc[2,7]+FF28
    FF37 = -s.frc[3,7]-FF18*S8+FF38*C8
    CF17 = -s.trq[1,7]+CF18*C8+CF38*S8
    CF27 = -s.trq[2,7]+CF28-s.dpt[1,2]*(-FF18*S8+FF38*C8)
    CF37 = -s.trq[3,7]-CF18*S8+CF38*C8+FF28*s.dpt[1,2]
    FF16 = -s.frc[1,6]+FF17*C7+FF37*S7
    FF26 = -s.frc[2,6]+FF27
    FF36 = -s.frc[3,6]-FF17*S7+FF37*C7
    CF16 = -s.trq[1,6]+CF17*C7+CF37*S7
    CF26 = -s.trq[2,6]+CF27-s.dpt[1,1]*(-FF17*S7+FF37*C7)
    CF36 = -s.trq[3,6]-CF17*S7+CF37*C7+FF27*s.dpt[1,1]
    FF15 = FF16*C6+FF36*S6
    FF35 = -FF16*S6+FF36*C6
    CF15 = CF16*C6+CF36*S6
    CF35 = -CF16*S6+CF36*C6
    FF24 = FF26*C5-FF35*S5
    FF34 = FF26*S5+FF35*C5
    CF34 = CF26*S5+CF35*C5
    FF13 = FF15*C4-FF24*S4
    FF23 = FF15*S4+FF24*C4
 
# Symbolic model output

    c[1] = FF13
    c[2] = FF23
    c[3] = FF34
    c[4] = CF34
    c[5] = CF15
    c[6] = CF26
    c[7] = CF27
    c[8] = CF28
    c[9] = CF29
    c[10] = FF110
    c[11] = CF211
    c[12] = CF212
    c[13] = CF313
    c[14] = FF314
    c[15] = -s.trq[2,15]
    c[16] = -s.frc[1,16]
    c[17] = CF217
    c[18] = CF218
    c[19] = -s.trq[2,19]
    c[20] = CF220
    c[21] = CF221
    c[22] = -s.trq[2,22]

# Number of continuation lines = 0


