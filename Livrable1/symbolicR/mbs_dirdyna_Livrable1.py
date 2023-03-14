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
#	==> Function: F1 - Recursive Direct Dynamics of tree-like MBS
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def dirdyna(M, c, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
 
# Forward Kinematics

    BS12 = -qd[2]*qd[2]
    BS92 = -qd[2]*qd[2]
    AF12 = s.g[3]*S2
    AF32 = -s.g[3]*C2
    OM23 = qd[2]+qd[3]
    BS13 = -OM23*OM23
    BS93 = -OM23*OM23
    AF13 = C3*(AF12+BS12*s.dpt[1,5])-S3*(AF32+BS92*s.dpt[3,5])
    AF33 = C3*(AF32+BS92*s.dpt[3,5])+S3*(AF12+BS12*s.dpt[1,5])
    AM13_1 = -C2*S3-S2*C3
    AM33_1 = C2*C3-S2*S3
    AM13_2 = s.dpt[1,5]*S3+s.dpt[3,5]*C3
    AM33_2 = -s.dpt[1,5]*C3+s.dpt[3,5]*S3
    AF14 = AF13*C4-S4*(AF33+BS93*s.dpt[3,6])
    AF34 = AF13*S4+C4*(AF33+BS93*s.dpt[3,6])
    AM14_1 = AM13_1*C4-AM33_1*S4
    AM34_1 = AM13_1*S4+AM33_1*C4
    AM14_2 = -AM33_2*S4+C4*(AM13_2+s.dpt[3,6])
    AM34_2 = AM33_2*C4+S4*(AM13_2+s.dpt[3,6])
    AM14_3 = s.dpt[3,6]*C4
    AM34_3 = s.dpt[3,6]*S4
    BS15 = -qd[5]*qd[5]
    BS95 = -qd[5]*qd[5]
    AF15 = s.g[3]*S5
    AF35 = -s.g[3]*C5
 
# Backward Dynamics

    FA15 = -s.frc[1,5]+s.m[5]*(AF15+BS15*s.l[1,5])
    FA35 = -s.frc[3,5]+s.m[5]*(AF35+BS95*s.l[3,5])
    CF25 = -s.trq[2,5]+FA15*s.l[3,5]-FA35*s.l[1,5]
    FB15_1 = -s.m[5]*S5
    FB35_1 = s.m[5]*C5
    CM25_1 = FB15_1*s.l[3,5]-FB35_1*s.l[1,5]
    FB15_5 = s.m[5]*s.l[3,5]
    FB35_5 = -s.m[5]*s.l[1,5]
    CM25_5 = s.In[5,5]+FB15_5*s.l[3,5]-FB35_5*s.l[1,5]
    FA14 = -s.frc[1,4]+s.m[4]*AF14
    FA34 = -s.frc[3,4]+s.m[4]*AF34
    FB14_1 = s.m[4]*AM14_1
    FB34_1 = s.m[4]*AM34_1
    FB14_2 = s.m[4]*AM14_2
    FB34_2 = s.m[4]*AM34_2
    FB14_3 = s.m[4]*AM14_3
    FB34_3 = s.m[4]*AM34_3
    FA13 = -s.frc[1,3]+s.m[3]*(AF13+BS13*s.l[1,3])
    FA33 = -s.frc[3,3]+s.m[3]*(AF33+BS93*s.l[3,3])
    FF13 = FA13+FA14*C4+FA34*S4
    FF33 = FA33-FA14*S4+FA34*C4
    CF23 = -s.trq[2,3]-s.trq[2,4]+FA13*s.l[3,3]-FA33*s.l[1,3]+s.dpt[3,6]*(FA14*C4+FA34*S4)
    FB13_1 = s.m[3]*AM13_1
    FB33_1 = s.m[3]*AM33_1
    FM13_1 = FB13_1+FB14_1*C4+FB34_1*S4
    FM33_1 = FB33_1-FB14_1*S4+FB34_1*C4
    CM23_1 = FB13_1*s.l[3,3]-FB33_1*s.l[1,3]+s.dpt[3,6]*(FB14_1*C4+FB34_1*S4)
    FB13_2 = s.m[3]*(AM13_2+s.l[3,3])
    FB33_2 = s.m[3]*(AM33_2-s.l[1,3])
    FM13_2 = FB13_2+FB14_2*C4+FB34_2*S4
    FM33_2 = FB33_2-FB14_2*S4+FB34_2*C4
    CM23_2 = s.In[5,3]+s.In[5,4]+FB13_2*s.l[3,3]-FB33_2*s.l[1,3]+s.dpt[3,6]*(FB14_2*C4+FB34_2*S4)
    FB13_3 = s.m[3]*s.l[3,3]
    FB33_3 = -s.m[3]*s.l[1,3]
    CM23_3 = s.In[5,3]+s.In[5,4]+FB13_3*s.l[3,3]-FB33_3*s.l[1,3]+s.dpt[3,6]*(FB14_3*C4+FB34_3*S4)
    FA12 = -s.frc[1,2]+s.m[2]*(AF12+BS12*s.l[1,2])
    FA32 = -s.frc[3,2]+s.m[2]*(AF32+BS92*s.l[3,2])
    FF12 = FA12+FF13*C3+FF33*S3
    FF32 = FA32-FF13*S3+FF33*C3
    CF22 = -s.trq[2,2]+CF23+FA12*s.l[3,2]-FA32*s.l[1,2]-s.dpt[1,5]*(-FF13*S3+FF33*C3)+s.dpt[3,5]*(FF13*C3+FF33*S3)
    FB12_1 = -s.m[2]*S2
    FB32_1 = s.m[2]*C2
    FM12_1 = FB12_1+FM13_1*C3+FM33_1*S3
    FM32_1 = FB32_1-FM13_1*S3+FM33_1*C3
    CM22_1 = CM23_1+FB12_1*s.l[3,2]-FB32_1*s.l[1,2]-s.dpt[1,5]*(-FM13_1*S3+FM33_1*C3)+s.dpt[3,5]*(FM13_1*C3+FM33_1*S3)
    FB12_2 = s.m[2]*s.l[3,2]
    FB32_2 = -s.m[2]*s.l[1,2]
    CM22_2 = s.In[5,2]+CM23_2+FB12_2*s.l[3,2]-FB32_2*s.l[1,2]-s.dpt[1,5]*(-FM13_2*S3+FM33_2*C3)+s.dpt[3,5]*(FM13_2*C3+FM33_2*S3)
    FA31 = -s.frc[3,1]-s.m[1]*s.g[3]
    FF31 = FA31-FA15*S5+FA35*C5-FF12*S2+FF32*C2
    FM31_1 = s.m[1]-FB15_1*S5+FB35_1*C5-FM12_1*S2+FM32_1*C2
 
# Symbolic model output

    c[1] = FF31
    c[2] = CF22
    c[3] = CF23
    c[4] = -s.trq[2,4]
    c[5] = CF25
    M[1,1] = FM31_1
    M[1,2] = CM22_1
    M[1,3] = CM23_1
    M[1,5] = CM25_1
    M[2,1] = CM22_1
    M[2,2] = CM22_2
    M[2,3] = CM23_2
    M[2,4] = s.In[5,4]
    M[3,1] = CM23_1
    M[3,2] = CM23_2
    M[3,3] = CM23_3
    M[3,4] = s.In[5,4]
    M[4,2] = s.In[5,4]
    M[4,3] = s.In[5,4]
    M[4,4] = s.In[5,4]
    M[5,1] = CM25_1
    M[5,5] = CM25_5

# Number of continuation lines = 0


