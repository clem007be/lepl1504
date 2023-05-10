# -*- coding: utf-8 -*-
"""Module for the definition of driven joints."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

from MBsysPy import MbsSensor
from numpy import *

def user_DrivenJoints(mbs_data, tsim):
    """Set the values of the driven joints directly in the MbsData structure.

    The position, velocity and acceleration of the driven joints must be set in
    the attributes mbs_data.q, mbs_data.qd and mbs_data.qdd .

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Returns
    -------
    None
    """
    
    # Cadre Vélo vitesse constante
    id_T1 = mbs_data.joint_id['T1Frame']
    id_T2 = mbs_data.joint_id['T2Frame']
    id_R3 = mbs_data.joint_id['R3Frame']
    
    p0_T1 = mbs_data.q0[id_T1]
    v0_T1 = mbs_data.qd0[id_T1]
    a0_T1 = mbs_data.qdd0[id_T1]
    p0_T2 = mbs_data.q0[id_T2]
    v0_T2 = mbs_data.qd0[id_T2]
    a0_T2 = mbs_data.qdd0[id_T2]
    
    theta = mbs_data.q[id_R3]
    a = sqrt(a0_T1**2 + a0_T2**2)
    v = sqrt((v0_T1 + a0_T1*tsim)**2 + (v0_T2 + a0_T2*tsim)**2)
    p = sqrt((p0_T1 + v0_T1*tsim + a0_T1/2 * tsim**2)**2 + (p0_T2 + v0_T2 * tsim - a0_T2/2 * tsim**2)**2)
    
    mbs_data.q[id_T1] = p * cos(theta)
    mbs_data.qd[id_T1] = v * cos(theta)
    mbs_data.qdd[id_T1] = a * cos(theta)
    # mbs_data.q[id_T2] = p * sin(theta)
    # mbs_data.qd[id_T2] = v * sin(theta)
    # mbs_data.qdd[id_T2] = a * sin(theta)
    
    # Angle Alpha
    id_j = mbs_data.joint_id['Alpha']
    mbs_data.q[id_j] = mbs_data.q0[id_j]
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    
    # Joint Chassis
    id_j = mbs_data.joint_id['Joint_Chassis']
    mbs_data.q[id_j] = mbs_data.q0[id_j]
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    
    # Joint Box
    id_j = mbs_data.joint_id['Joint_Box']
    mbs_data.q[id_j] = mbs_data.q0[id_j]
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    
    # Inclinaison fourche
    id_j = mbs_data.joint_id['Attache_Fourche']
    mbs_data.q[id_j] = mbs_data.q0[id_j]
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    
    #
    id_j = mbs_data.joint_id['R2_Fourche_Rem']
    mbs_data.q[id_j] = mbs_data.q0[id_j]
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    
    # Attache
    id_j = mbs_data.joint_id['Attache12']
    mbs_data.q[id_j] = mbs_data.q0[id_j]
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    id_j = mbs_data.joint_id['Attache23']
    mbs_data.q[id_j] = mbs_data.q0[id_j]
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    
    return
