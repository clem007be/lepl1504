# -*- coding: utf-8 -*-
"""Module for the definition of driven joints."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


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
    
    # Cadre Vélo
    id_j = mbs_data.joint_id['T1Frame']
    p0 = mbs_data.q0[id_j]
    v0 = mbs_data.qd0[id_j]
    a0 = mbs_data.qdd0[id_j]
    mbs_data.q[id_j] = p0 + v0*tsim + a0/2 * tsim**2
    mbs_data.qd[id_j] = v0 + a0 * tsim
    mbs_data.qdd[id_j] = a0
    
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
