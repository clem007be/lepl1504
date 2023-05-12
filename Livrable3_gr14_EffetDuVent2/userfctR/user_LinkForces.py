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
    
    # Guidon droit
    link_id = mbs_data.link_id['Link_GuidonGauche']
    if (identity == link_id):
        sensor_FWheel = MbsSensor(mbs_data)
        sensor_FWheel.comp_s_sensor(mbs_data.sensor_id['Sensor_FWheel'])
        if(sensor_FWheel.P[3] < mbs_data.user_model['roue']['R0']):
            R1Frame = mbs_data.joint_id['R1Frame']
            phi = mbs_data.q[R1Frame]
            phid = mbs_data.qd[R1Frame]
            K_phi = 100
            K_phid = 125
            
            #Evitement /!\/!\/!\ mutuellement EXCLUSIF avec Demi-tour !!!
            T2Frame = mbs_data.joint_id['T2Frame']
            T1Frame = mbs_data.joint_id['T1Frame']
            y = mbs_data.q[T2Frame]
            yd = mbs_data.qd[T2Frame]
            T2 = .38
            T2_d = .45
            x_init = 4      # distance du départ de la manoeuvre
            x_max = 15      # distance à laquelle le rabattement commence 
            if (mbs_data.q[T1Frame] > x_init and mbs_data.q[T1Frame] < x_max):
                y0 = 2
            else:
                y0 = 0
            phi0 = T2 * (y-y0) + T2_d * yd
            
            #Demi-tour !!! mutuellement EXCLUSIF avec Evitement !!!
            # R3Frame = mbs_data.joint_id['R3Frame']
            # theta = mbs_data.q[R3Frame]
            # theta0 = -np.pi
            # phi0 = 0
            # if(theta > theta0 and tsim > 1):
            #     phi0 = 0.3        # angle de roulis pour le demi-tour 
            #                       # [0.3, 0.4, 0.5, 0.6] autres valeurs analysées 
            
            
            Flink = - K_phi * (phi - phi0) - K_phid * phid
            
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
