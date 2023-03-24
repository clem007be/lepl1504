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
#	==> Function: F8 - Constraints and Constraints Jacobian(h, J)
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def cons_hJ(h, Jac, s):
    q = s.q
 
# Trigonometric functions

    S17 = sin(q[17])
    C17 = cos(q[17])
    S20 = sin(q[20])
    C20 = cos(q[20])
 
# Augmented Joint Position Vectors

    Dz101 = q[10]+s.dpt[1,4]
    Dz143 = q[14]+s.dpt[3,8]
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

    POlp1_12 = q[16]+s.dpt[1,11]
    RLlp2_12 = s.dpt[1,14]*C17+s.dpt[3,14]*S17
    RLlp2_32 = -s.dpt[1,14]*S17+s.dpt[3,14]*C17
    POlp2_32 = RLlp2_32+s.dpt[3,6]
    h_1 = POlp1_12-RLlp2_12
    h_3 = -POlp2_32
    POlp3_12 = q[16]+s.dpt[1,12]
    RLlp4_12 = s.dpt[1,18]*C20+s.dpt[3,18]*S20
    RLlp4_32 = -s.dpt[1,18]*S20+s.dpt[3,18]*C20
    POlp4_32 = RLlp4_32+s.dpt[3,7]
    h_7 = POlp3_12-RLlp4_12
    h_9 = -POlp4_32
    h[1] = h_1
    h[2] = h_3
    h[3] = -S17
    h[4] = h_7
    h[5] = h_9
    h[6] = -S20
    Jac[1,1] = 0
    Jac[1,2] = 0
    Jac[1,3] = 0
    Jac[1,4] = 0
    Jac[1,5] = 0
    Jac[1,6] = 0
    Jac[1,7] = 0
    Jac[1,8] = 0
    Jac[1,9] = 0
    Jac[1,10] = 0
    Jac[1,11] = 0
    Jac[1,12] = 0
    Jac[1,13] = 0
    Jac[1,14] = 0
    Jac[1,15] = 0
    Jac[1,16] = (1.0)
    Jac[1,17] = -RLlp2_32
    Jac[1,18] = 0
    Jac[1,19] = 0
    Jac[1,20] = 0
    Jac[1,21] = 0
    Jac[1,22] = 0
    Jac[2,1] = 0
    Jac[2,2] = 0
    Jac[2,3] = 0
    Jac[2,4] = 0
    Jac[2,5] = 0
    Jac[2,6] = 0
    Jac[2,7] = 0
    Jac[2,8] = 0
    Jac[2,9] = 0
    Jac[2,10] = 0
    Jac[2,11] = 0
    Jac[2,12] = 0
    Jac[2,13] = 0
    Jac[2,14] = 0
    Jac[2,15] = 0
    Jac[2,16] = 0
    Jac[2,17] = RLlp2_12
    Jac[2,18] = 0
    Jac[2,19] = 0
    Jac[2,20] = 0
    Jac[2,21] = 0
    Jac[2,22] = 0
    Jac[3,1] = 0
    Jac[3,2] = 0
    Jac[3,3] = 0
    Jac[3,4] = 0
    Jac[3,5] = 0
    Jac[3,6] = 0
    Jac[3,7] = 0
    Jac[3,8] = 0
    Jac[3,9] = 0
    Jac[3,10] = 0
    Jac[3,11] = 0
    Jac[3,12] = 0
    Jac[3,13] = 0
    Jac[3,14] = 0
    Jac[3,15] = 0
    Jac[3,16] = 0
    Jac[3,17] = -(1.0)
    Jac[3,18] = 0
    Jac[3,19] = 0
    Jac[3,20] = 0
    Jac[3,21] = 0
    Jac[3,22] = 0
    Jac[4,1] = 0
    Jac[4,2] = 0
    Jac[4,3] = 0
    Jac[4,4] = 0
    Jac[4,5] = 0
    Jac[4,6] = 0
    Jac[4,7] = 0
    Jac[4,8] = 0
    Jac[4,9] = 0
    Jac[4,10] = 0
    Jac[4,11] = 0
    Jac[4,12] = 0
    Jac[4,13] = 0
    Jac[4,14] = 0
    Jac[4,15] = 0
    Jac[4,16] = (1.0)
    Jac[4,17] = 0
    Jac[4,18] = 0
    Jac[4,19] = 0
    Jac[4,20] = -RLlp4_32
    Jac[4,21] = 0
    Jac[4,22] = 0
    Jac[5,1] = 0
    Jac[5,2] = 0
    Jac[5,3] = 0
    Jac[5,4] = 0
    Jac[5,5] = 0
    Jac[5,6] = 0
    Jac[5,7] = 0
    Jac[5,8] = 0
    Jac[5,9] = 0
    Jac[5,10] = 0
    Jac[5,11] = 0
    Jac[5,12] = 0
    Jac[5,13] = 0
    Jac[5,14] = 0
    Jac[5,15] = 0
    Jac[5,16] = 0
    Jac[5,17] = 0
    Jac[5,18] = 0
    Jac[5,19] = 0
    Jac[5,20] = RLlp4_12
    Jac[5,21] = 0
    Jac[5,22] = 0
    Jac[6,1] = 0
    Jac[6,2] = 0
    Jac[6,3] = 0
    Jac[6,4] = 0
    Jac[6,5] = 0
    Jac[6,6] = 0
    Jac[6,7] = 0
    Jac[6,8] = 0
    Jac[6,9] = 0
    Jac[6,10] = 0
    Jac[6,11] = 0
    Jac[6,12] = 0
    Jac[6,13] = 0
    Jac[6,14] = 0
    Jac[6,15] = 0
    Jac[6,16] = 0
    Jac[6,17] = 0
    Jac[6,18] = 0
    Jac[6,19] = 0
    Jac[6,20] = -(1.0)
    Jac[6,21] = 0
    Jac[6,22] = 0

# Number of continuation lines = 0


