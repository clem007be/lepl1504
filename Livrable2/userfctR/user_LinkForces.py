# -*- coding: utf-8 -*-
"""Module for the definition of user links forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

from MBsysPy import MbsSensor

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
    link_id = mbs_data.link_id['Link_Guidon']
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
            K_phi = 80
            K_phid = 125
            Flink = - K_phi * (phi - phi0) - K_phid * phid

    # Example: linear spring
    # k = 1000 #N/m
    # Z0= 0.1  #m
    # Flink = k*(Z-Z0)

    return Flink
