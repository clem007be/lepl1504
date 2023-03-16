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

    # Example: joint 5 under constant acceleration with non-zero initial
    #          coordinate (mbs_data.q0) and velocity (mbs_data.qd0).
    # mbs_data.qdd[5] = 2
    # mbs_data.qd[5]  = mbs_data.qd0[5] + mbs_data.qdd[5]*tsim
    # mbs_data.q[5]   = mbs_data.q0[5]  + mbs_data.qd0[5]*tsim + 0.5 * mbs_data.qdd[5]*tsim*tsim
    
    # #T1JOINT_0
    # # get the joint id
    # id_j0 = mbs_data.joint_id["Joint_0"]
    # v0 = 30
    # #omega = -2.0*np.pi*0.5    
    # # impose the position, velocity and acceleration
    # mbs_data.q[id_j0]   = v0*tsim + mbs_data.q0[id_j0] # On met le v0 à 30 m/s
    # mbs_data.qd[id_j0]  = v0
    # mbs_data.qdd[id_j0] = 0
    
    # #RWHEEL1  OMEGA = Vduvelo/Rayonroue
    # # get the joint id
    # id_j1 = mbs_data.joint_id["R2Wheel1"]
    # Rayon1 = 0.3 #[m]
    # omega1 = -mbs_data.qd[id_j0]/Rayon1
    # # impose the position, velocity and acceleration
    # mbs_data.q[id_j1]   = omega1*tsim
    # mbs_data.qd[id_j1]  = omega1
    # mbs_data.qdd[id_j1] = 0
    
    #Cadre Vélo
    id_j = mbs_data.joint_id['T1Frame']
    mbs_data.q[id_j] = tsim*1
    mbs_data.qd[id_j] = 1
    mbs_data.qdd[id_j] = 0
    
    #Angle Alpha
    id_j = mbs_data.joint_id['Alpha']
    mbs_data.q[id_j] = -0.4
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    
    #Tournant de la fourche
    id_j = mbs_data.joint_id['R3Fourche']
    mbs_data.q[id_j] = 0
    mbs_data.qd[id_j] = 0
    mbs_data.qdd[id_j] = 0
    
    # #RWHEEL
    # # get the joint id
    # id_j = mbs_data.joint_id["R2RWheel"]
    # R = mbs_data.user_model['roue']['R0']
    # V = 10
    # Omega = V/R
    # # impose the position, velocity and acceleration
    # mbs_data.q[id_j]   = Omega*tsim
    # mbs_data.qd[id_j]  = Omega
    # mbs_data.qdd[id_j] = 0

    return
