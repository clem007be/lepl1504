# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019
from mbs_tgc import *
import numpy as np

def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    """Compute an user-specified external force.

    Parameters
    ----------
    PxF : numpy.ndarray
        Position vector (index starting at 1) of the force sensor expressed in
        the inertial frame: PxF[1:4] = [P_x, P_y, P_z]
    RxF : numpy.ndarray
        Rotation matrix (index starting at 1) from the inertial frame to the
        force sensor frame: Frame_sensor = RxF[1:4,1:4] * Frame_inertial
    VxF : numpy.ndarray
        Velocity vector (index starting at 1) of the force sensor expressed in
        the inertial frame: VxF[1:4] = [V_x, V_y, V_z]
    OMxF : numpy.ndarray
        Angular velocity vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMxF[1:4] = [OM_x, OM_y, OM_z]
    AxF : numpy.ndarray
        Acceleration vector (index starting at 1) of the force sensor expressed
        in the inertial frame: AxF[1:4] = [A_x, A_y, A_z]
    OMPxF : numpy.ndarray
        Angular acceleration vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMPxF[1:4] = [OMP_x, OMP_y, OMP_z]
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    ixF : int
        The ID identifying the computed force sensor.

    Notes
    -----
    For 1D numpy.ndarray with index starting at 1, the first index (array[0])
    must not be modified. The first index to be filled is array[1].

    For 2D numpy.ndarray with index starting at 1, the first row (mat[0, :]) and
    line (mat[:,0]) must not be modified. The subarray to be filled is mat[1:, 1:].

    Returns
    -------
    Swr : numpy.ndarray
        An array of length 10 equal to [0., Fx, Fy, Fz, Mx, My, Mz, dxF].
        F_# are the forces components expressed in inertial frame.
        M_# are the torques components expressed in inertial frame.
        dxF is an array of length 3 containing the component of the forces/torque
        application point expressed in the BODY-FIXED frame.
        This array is a specific line of MbsData.SWr.
    """

    Fx = 0.0
    Fy = 0.0
    Fz = 0.0
    Mx = 0.0
    My = 0.0
    Mz = 0.0
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]
    verbose = False
    R0 = mbs_data.user_model['roue']['R0']
    
    tgc = mbs_tgc.tgc_car_kine_wheel(PxF, RxF, VxF, OMxF, R0)
    
    #Transformation en dictionnaire pour une utilisation plus simple ===========
    arg = ['pen','rz','angslip','angcamb','slip','Pct','Vmct','Rt_ground','dxF']
    tgc_dic = {}
    for i in range(len(tgc)):
        tgc_dic[arg[i]] = tgc[i] 
    #==========================================================================
    
    if(tgc_dic['pen'] > 0):
        K = mbs_data.user_model['roue']['K']
        D = mbs_data.user_model['roue']['D']
        dedt = np.dot(tgc_dic['Rt_ground'],tgc_dic['Vmct'])
        Fz = K*tgc_dic['pen']-D*dedt[3]
        
    
    
    # pen = tgc[0]
    # rz = tgc[1]
    # angslip = tgc[2]
    # angcamb = tgc[3]
    # slip = tgc[4] 
    # RotMTX = tgc[7]
    # mbs_tgc.tgc_bakker_contact(FWhl,MWhl,angslip,angcamb,slip)
    
    # FWhl = np.dot(RotMTX,FWhl)
    # RWhl = np.dot(RotMTX,MWhl)
    
    # Fx = FWhl[1]
    # Fy = FWhl[2]
    # Fz = FWhl[3]
    # Mx = MWhl[1]
    # My = MWhl[2]
    # Mz = MWhl[3]
    # dxF = tgc[8][1:]
    if(verbose):
        try :
            f = open('../analyse/analyse.txt','a')
            f.write("{}\n\n".format(tgc_dic))
            f.close()
        except :
            print("unable to open the file") 
    
    # Concatenating force, torque and force application point to returned array.
    # This must not be modified.
    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [Fx, Fy, Fz, Mx, My, Mz, dxF[0], dxF[1], dxF[2]]

    return Swr
