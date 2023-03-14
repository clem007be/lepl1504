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
#	==> Function: F8 - Constraints and Constraints Jacobian(h, J)
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def cons_hJ(h, Jac, s):
    q = s.q
 
# Trigonometric functions

    S5 = sin(q[5])
    C5 = cos(q[5])
    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

    RLlp1_12 = s.dpt[1,10]*C5+s.dpt[3,10]*S5
    RLlp1_32 = -s.dpt[1,10]*S5+s.dpt[3,10]*C5
    POlp1_12 = RLlp1_12+s.dpt[1,3]
    POlp1_32 = RLlp1_32+s.dpt[3,3]
    ROlp2_12 = C2*C3-S2*S3
    ROlp2_32 = -C2*S3-S2*C3
    RLlp2_12 = s.dpt[1,5]*C2+s.dpt[3,5]*S2
    RLlp2_32 = -s.dpt[1,5]*S2+s.dpt[3,5]*C2
    POlp2_12 = RLlp2_12+s.dpt[1,1]
    POlp2_32 = RLlp2_32+s.dpt[3,1]
    RLlp2_13 = ROlp2_12*s.dpt[1,7]
    RLlp2_33 = ROlp2_32*s.dpt[1,7]
    POlp2_13 = POlp2_12+RLlp2_13
    POlp2_33 = POlp2_32+RLlp2_33
    JTlp2_13_1 = RLlp2_32+RLlp2_33
    JTlp2_33_1 = -RLlp2_12-RLlp2_13
    h_1 = POlp1_12-POlp2_13
    h_3 = POlp1_32-POlp2_33
    h[1] = h_1
    h[2] = h_3
    Jac[1,1] = 0
    Jac[1,2] = -JTlp2_13_1
    Jac[1,3] = -RLlp2_33
    Jac[1,4] = 0
    Jac[1,5] = RLlp1_32
    Jac[2,1] = 0
    Jac[2,2] = -JTlp2_33_1
    Jac[2,3] = RLlp2_13
    Jac[2,4] = 0
    Jac[2,5] = -RLlp1_12

# Number of continuation lines = 0


