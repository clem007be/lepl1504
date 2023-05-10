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
                      "See: https://www.robotran.eu/download/how-to-install/"
                      )
# %%===========================================================================
# Project loading
# =============================================================================
mbs_data = Robotran.MbsData('../dataR/Livrable2.mbs')

T1Frame = mbs_data.joint_id['T1Frame']
T2Frame = mbs_data.joint_id['T2Frame']
R3Frame = mbs_data.joint_id['R3Frame']
R2RWheel = mbs_data.joint_id['R2RWheel']
R2FWheel = mbs_data.joint_id['R2FWheel']
R2_FWheel_Rem = mbs_data.joint_id['R2_FWheel_Rem']
R2_RoueG = mbs_data.joint_id['R2_RoueG']
R2_RoueD = mbs_data.joint_id['R2_RoueD']

theta = 0 # rad
vitesse = 5 # m/s

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
mbs_dirdyn.set_options(dt0=1e-3, tf=10.0, save2file=1)
results = mbs_dirdyn.run()

# %%===========================================================================
# Plotting results
# =============================================================================

# np.savetxt('../analyse/q5/v/stabilite/roulis0,2_v{}.txt'.format(i), np.array([results.q[:,5], results.qd[:,5]]))

# Figure creation
dic = mbs_data.joint_id
fig = plt.figure(num='R3Fourche')
fig.set_tight_layout(True)
gs = gridspec.GridSpec(1,1)
id_j = mbs_data.joint_id['R3Fourche']
axis = fig.add_subplot(gs[0,0])
axis.plot(results.q[:, 0], results.q[:, id_j])
axis.grid(True)
axis.set_xlabel('Time (s)')
axis.set_ylabel('Coordinate value (m or rad)')
# Plotting data's
# for i in dic:
#     id_j = mbs_data.joint_id[i]
#     axis = fig.add_subplot(gs[(id_j-1)//5, (id_j-1)%5])
#     axis.plot(results.q[:, 0], results.q[:, id_j])
    
#     axis.set_title('{}'.format(i))
#     axis.grid(True)
#     axis.set_xlabel('Time (s)')
#     axis.set_ylabel('Coordinate value (m or rad)')
# axis = fig.add_subplot(gs[:,:])
# axis.plot(results.q[:,0], results.q[:,9])
# axis.set_xlabel('Temps (s)')
# axis.set_ylabel('Angle (rad)')
# axis.grid(True)

# plt.savefig('../graphe/Inclinaison_q5.svg')

fig3 = plt.figure(num="déplacement latéral")
fig3.set_tight_layout(True)
gs = gridspec.GridSpec(3,1)
axis = fig3.add_subplot(gs[0,0])
axis.plot(results.t, results.outputs['ChassisP2'])
axis.set_xlabel('Temps (s)')
axis.set_ylabel('Distance en Y (m)')
axis.grid(True)

axis = fig3.add_subplot(gs[1,0])
axis.plot(results.t, results.outputs['ChassisV1'])
axis.set_xlabel('Temps (s)')
axis.set_ylabel('Vitesse en x (m/s)')
axis.grid(True)

axis = fig3.add_subplot(gs[2,0])
axis.plot(results.t, results.outputs['ChassisV2'])
axis.set_xlabel('Temps (s)')
axis.set_ylabel('Vitesse en y (m/s)')
axis.grid(True)

# plt.savefig('../graphe/Inclinaison_qd5.svg')

# Figure enhancement
# axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))

fig4 = plt.figure(num="R3Frame")
axis = fig4.gca()
axis.plot(results.q[:,0], results.q[:,4])
axis.set_xlabel('Temps (s)')
axis.set_ylabel('R3Frame (rad)')
axis.grid(True)

fig2 = plt.figure(num="Force Guidon Gauche")
axis = fig2.gca()
id_link = mbs_data.link_id['Link_GuidonGauche']
axis.plot(results.t, results.Fl[:,id_link])
axis.set_xlabel('Temps (s)')
axis.set_ylabel('Force (N)')
axis.grid(True)

# plt.savefig('../graphe/FLink_Guidon.svg')

plt.show()

