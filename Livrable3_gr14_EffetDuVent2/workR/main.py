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
    import winsound as ws

    
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/")
    
# %%===========================================================================
# Project loading
# =============================================================================
try:
    v = [15] #[12,15,18,20]
    for i in v:
        print(i)
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
        vitesse = i/3.6 # m/s
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
        mbs_dirdyn.set_options(dt0=1e-3, tf=10.0, save2file=1)
        results = mbs_dirdyn.run()
        
        # %%===========================================================================
        # Plotting results
        # =============================================================================
        
        # np.savetxt('../analyse/Demi-tour/v{}/roulis{}_m{}.txt'.format(i,0.6,m), np.array([results.outputs["ChassisP1"], results.outputs["ChassisP2"]]))
        
        # Figure creation
        # dic = mbs_data.joint_id
        fig = plt.figure()
        # axis = fig.gca()
        # # fig.set_tight_layout(True)
        gs = gridspec.GridSpec(6,5)
        axis = fig.add_subplot(gs[:,:])
        axis.plot(results.outputs["ChassisP2"], results.outputs["ChassisP1"])
        axis.set_aspect('equal')
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
        
        plt.show()
        
except:
    raise RuntimeError("Ca va pas")
finally:
    ws.PlaySound('../Messenger.wav', ws.SND_FILENAME)

