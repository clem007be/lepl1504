#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Script to run a direct dynamic analysis on a multibody system.

Summary
-------
This template loads the data file *.mbs and execute:
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - if available, plot the time evolution of the first generalized coordinate.

It may have to be adapted and completed by the user.


Universite catholique de Louvain
CEREM : Centre for research in mechatronics

http://www.robotran.eu
Contact : info@robotran.be

(c) Universite catholique de Louvain
"""

# %%============================================================================
# Packages loading
# =============================================================================
try:
    import MBsysPy as Robotran
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/")
    
# %%===========================================================================
# Project loading
# =============================================================================
masse = [0.01, 32.5, 130.0] # [kg]      valeurs utilisées 
v = [12,15,18,20]           # [km/h]    pour analyser le comportement du mobile

mbs_data = Robotran.MbsData('../dataR/Livrable2.mbs')

T1Frame = mbs_data.joint_id['T1Frame']
T2Frame = mbs_data.joint_id['T2Frame']
R3Frame = mbs_data.joint_id['R3Frame']
R2RWheel = mbs_data.joint_id['R2RWheel']
R2FWheel = mbs_data.joint_id['R2FWheel']
R2_FWheel_Rem = mbs_data.joint_id['R2_FWheel_Rem']
R2_RoueG = mbs_data.joint_id['R2_RoueG']
R2_RoueD = mbs_data.joint_id['R2_RoueD']
ChargeAv = mbs_data.body_id['Chargement_Avant']
ChargeM = mbs_data.body_id['Chargement_Milieu']
ChargeAr = mbs_data.body_id['Chargement_Arriere']

theta = 0 # rad
vitesse = v[1]/3.6 # m/s
m = mbs_data.m[ChargeAv]+ mbs_data.m[ChargeM] + mbs_data.m[ChargeAr]

mbs_data.q0[R3Frame] = theta
mbs_data.qd0[T1Frame] = vitesse * np.cos(theta)
mbs_data.qd0[T2Frame] = vitesse * np.sin(theta)
mbs_data.qd0[R2RWheel] = vitesse/0.35
mbs_data.qd0[R2FWheel] = vitesse/0.35
mbs_data.qd0[R2_FWheel_Rem] = vitesse/0.2
mbs_data.qd0[R2_RoueG] = vitesse/0.2
mbs_data.qd0[R2_RoueD] = vitesse/0.2


#cleaning file analyse.txt
# try:
#     f = open('../analyse/analyse_RWheel.txt','w')
#     f.close()
#     f = open('../analyse/analyse_FWheel.txt','w')
#     f.close()
#     f = open('../analyse/Force_FWheel.txt','w')
#     f.close()
#     f = open('../analyse/Force_RWheel.txt','w')
#     f.close()
# except:
#     print('unable to open file')
# %%===========================================================================
# Partitionning
# =============================================================================
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# %%===========================================================================
# Direct Dynamics
# =============================================================================
mbs_data.process = 3
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
mbs_dirdyn.set_options(dt0=1e-3, tf=12.0, save2file=1)
results = mbs_dirdyn.run()

# %%===========================================================================
# Plotting results
# =============================================================================

# np.savetxt('../analyse/Evitement/v{}_m{}.txt'.format(i,m), np.array([results.outputs["ChassisP1"], results.outputs["ChassisP2"]]))

# Figure creation
# FGuidon = mbs_data.link_id['Link_GuidonGauche']
# fig = plt.figure(num="Guidage")
# fig.set_tight_layout(True)
# gs = gridspec.GridSpec(6,5)
# axis = fig.add_subplot(gs[:3,:])
# axis.plot(results.t, results.Fl[:,FGuidon])
# axis.grid(True)
# plt.title('FGuidon')
# # axis.set_aspect('equal')
# axis = fig.add_subplot(gs[3:,:])
# axis.plot(results.t, results.outputs["phi0"])
# plt.title('phi0')
# axis.grid(True)

fig = plt.figure(num="Trajectoire")
fig.set_tight_layout(True)
gs = gridspec.GridSpec(2,1)
axis = fig.add_subplot(gs[:,:])
axis.plot(results.outputs["ChassisP1"], results.outputs["ChassisP2"])
plt.title('Trajectoire')
axis.grid(True)
axis.set_aspect('equal','box')
# axis = fig.add_subplot(gs[1,0])
# axis.plot(results.qd[:,0], results.qd[:,2])
# plt.title('q2')
# axis.grid(True)

# fig = plt.figure(num="R1Frame")
# fig.set_tight_layout(True)
# gs = gridspec.GridSpec(2,1)
# axis = fig.add_subplot(gs[0,0])
# axis.plot(results.q[:,0], results.q[:,5])
# plt.title('q5')
# axis.grid(True)
# # axis.set_aspect('equal')
# axis = fig.add_subplot(gs[1,0])
# axis.plot(results.qd[:,0], results.qd[:,5])
# plt.title('qd5')
# axis.grid(True)
# axis.set_aspect('equal')
# a = mbs_data.qu
# for i in range(len(a)):
# # id_j = mbs_data.joint_id['R3Fourche']
#     axis = fig.add_subplot(gs[i//6,i%5])
#     axis.plot(results.q[:,0], results.q[:,a[i]])
#     # axis.set_xlim(1,-50)
#     # axis.set_aspect('equal','box')
#     plt.title('q{}'.format(a[i]))
#     axis.grid(True)
#     axis.set_xlabel('Time (s)')
#     axis.set_ylabel('Coordinate value (m or rad)')

# plt.show()
