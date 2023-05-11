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
    #Rear Drive
    id_j = mbs_data.joint_id['R2RWheel']
    mbs_data.q[id_j] = mbs_data.q0[id_j] + mbs_data.qd0[id_j]*tsim
    mbs_data.qd[id_j] = mbs_data.qd0[id_j]
    
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
    
    # Charge
    id_j = mbs_data.joint_id['Joint_ChargeAv']
    mbs_data.q[id_j] = mbs_data.q0[id_j]
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    id_j = mbs_data.joint_id['Joint_ChargeM']
    mbs_data.q[id_j] = mbs_data.q0[id_j]
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    id_j = mbs_data.joint_id['Joint_ChargeAr']
    mbs_data.q[id_j] = mbs_data.q0[id_j]
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    return
