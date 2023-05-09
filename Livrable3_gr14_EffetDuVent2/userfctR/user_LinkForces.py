# -*- coding: utf-8 -*-
"""Module for the definition of user links forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

from MBsysPy import MbsSensor
import numpy as np

def user_LinkForces(Z, Zd, mbs_data, tsim, identity):
    """Compute the force in the given link.

    Parameters
    ----------
    Z : float
        The distance between the two anchor points of the link.
    Zd : float
        The relative velocity between the two anchor points of the link.
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    identity : int
        The identity of the computed link.

    Returns
    -------
    Flink : float
        The force in the current link.

    """

    Flink = 0.0
    
    # Guidon Gauche
    link_id = mbs_data.link_id['Link_GuidonGauche']
    if (identity == link_id):
        sensor_FWheel = MbsSensor(mbs_data)
        sensor_FWheel.comp_s_sensor(mbs_data.sensor_id['Sensor_FWheel'])
        if(sensor_FWheel.P[3] < mbs_data.user_model['roue']['R0']):
            # Trajectoire
            T2_id = mbs_data.joint_id['T2Frame']
            T_y = 1
            T_yd = 1
            phi0 = 0 #- T_y * mbs_data.q[T2_id] - T_yd * mbs_data.qd[T2_id]
            # Gestion du moment à appliquer
            R1_id = mbs_data.joint_id['R1Frame']
            phi = mbs_data.q[R1_id]
            phid = mbs_data.qd[R1_id]
            R3_id = mbs_data.joint_id['R3Fourche']
            theta = mbs_data.q[R3_id]
            theta0 = np.pi/6
            if (theta != theta0 and (tsim > 1 and tsim < 3)):
                K_theta = 100
                Flink = - K_theta * (theta - theta0)
                #100 * np.exp(-(6 * (tsim - 1.5))**2)
            # elif (tsim > 3 and tsim < 4):
            #     mbs_data.q[R3_id] = -np.cos(w * (tsim - 4)) + 1
            #     Flink = 0
            else:
                K_phi = 30
                K_phid = 100
                Flink = - K_phi * (phi - phi0) - K_phid * phid
                
                
    #Guidon Droit
    link_id = mbs_data.link_id['Link_GuidonDroit']
    if (identity == link_id):
        Flink = 0
    #     sensor_FWheel = MbsSensor(mbs_data)
    #     sensor_FWheel.comp_s_sensor(mbs_data.sensor_id['Sensor_FWheel'])
    #     if(sensor_FWheel.P[3] < mbs_data.user_model['roue']['R0']):
    #         # Trajectoire
    #         T2_id = mbs_data.joint_id['T2Frame']
    #         T_y = 1
    #         T_yd = 1
    #         phi0 = 0 #- T_y * mbs_data.q[T2_id] - T_yd * mbs_data.qd[T2_id]
    #         # Gestion du moment à appliquer
    #         R1_id = mbs_data.joint_id['R1Frame']
    #         phi = mbs_data.q[R1_id]
    #         phid = mbs_data.qd[R1_id]
    #         R3_id = mbs_data.joint_id['R3Fourche']
    #         w = 0.13*np.pi
            
    #         if (tsim > 1 and tsim < 2 ):
    #             mbs_data.q[R3_id] = np.cos(w * (tsim - 1)) - 1
    #             Flink = - 5 * np.exp(-(6.5 * (tsim - 1.5)**2))
    #         # elif (tsim > 3 and tsim < 4):
    #         #     mbs_data.q[R3_id] = -np.cos(w * (tsim - 4)) + 1
    #         #     Flink = 0
    #         else:
    #             K_phi =30
    #             K_phid = 100
    #             Flink =  K_phi * (phi - phi0) + K_phid * phid
    
    # Amortisseur Arriere
    linkG_id = mbs_data.link_id['Amortisseur_RoueG']
    linkD_id = mbs_data.link_id['Amortisseur_RoueD']
    
    if (identity == linkG_id or identity == linkD_id):
        K = mbs_data.user_model['Amortisseur_Arriere']['K']
        D = mbs_data.user_model['Amortisseur_Arriere']['D']
        Z0 = mbs_data.user_model['Amortisseur_Arriere']['Z0']
        Flink = K*(Z-Z0) + D*Zd
    

    # Example: linear spring
    # k = 1000 #N/m
    # Z0= 0.1  #m
    # Flink = k*(Z-Z0)

    return Flink
