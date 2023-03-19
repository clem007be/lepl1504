# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019
from mbs_tgc import *
import numpy as np
import csv
from MBsysPy import matrix_vector_product

def verbose_tgc(boo, file, tgc_dic, ftype='txt'):
    if(boo):
        if(ftype == 'csv'):
            try :
                f = open(file,'a')
                writer = csv.writer(f, delimiter=',')
                # writer.writerow(tgc_dic[0:7])
                f.close()
            except :
                print("unable to open the file tgc.csv") 
        elif(ftype == 'txt'):
            try :
                f = open(file,'a')
                for i in tgc_dic:    
                    f.write('{} : {}\n'.format(i, tgc_dic[i]))
                f.write('\n')
                f.close()
            except :
                print("unable to open the file tgc.txt") 
            
def verbose_Force(boo,file,Force,ftype='txt'):
    if(boo):
        if ftype == 'csv':
            try:
                f = open(file,'a')
                writer = csv.writer(f, delimiter=',')
                writer.writerow(Force[1:])
                f.close()
            except :
                print('unable to open file force.csv')
        if ftype == 'txt':
            try:
                f = open(file,'a')
                f.write('Force : {:2e}, {:2e}, {:2e}\n'.format(Force[1],Force[2],Force[3]))
                f.write('\n')
                f.close()
            except :
                print('unable to open file force.txt')
            
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

    F = np.array([3.,0.,0.,0.])
    M = np.array([3.,0.,0.,0.])
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]
    
    analyseTGC = True
    analyseForce = True
    
    # Roue arrière ============================================================
    Fext = mbs_data.extforce_id["ExtForce_RWheel"]
    if ixF == Fext:
        
        R0 = mbs_data.user_model['roue']['R0']
    
        tgc = tgc_car_kine_wheel(PxF, RxF, VxF, OMxF, R0)
    
        # Transformation en dictionnaire pour une utilisation plus simple =====
        arg = ['pen','rz','angslip','angcamb','slip','Pct','Vmct','Rt_ground','dxF']
        tgc_dic = {}
        for i in range(len(tgc)):
            tgc_dic[arg[i]] = tgc[i] 
    
        # Force Normale =======================================================
        if(tgc_dic['pen'] > 0):
            K = mbs_data.user_model['roue']['K']
            D = mbs_data.user_model['roue']['D']
            dedt = np.dot(RxF,tgc_dic['Vmct'])
            F[3] = K*tgc_dic['pen']-D*dedt[3]
        
        # Force longitudinale =================================================
        # if (tgc_dic['pen'] > 0):
        #     CFx = mbs_data.user_model['roue']['CFx']
        #     Flong = CFx*tgc_dic['slip']
        #     Flong_I = np.array([3., Flong, 0., 0.])
        #     Flong_Y = matrix_vector_product(RxF, Flong_I)
        #     for i in range(1,4):
        #         F[i] += Flong_Y[i]
        
        dxF = tgc_dic['dxF'][1:]
        
        verbose_tgc(analyseTGC,'../analyse/analyse_RWheel.txt',tgc_dic,ftype='txt')
        verbose_Force(analyseForce,'../analyse/Force_RWheel.txt', F, ftype='txt')
    
    # Roue avant ==============================================================
    Fext = mbs_data.extforce_id["ExtForce_FWheel"]
    if ixF == Fext:
        
        R0 = mbs_data.user_model['roue']['R0']
    
        tgc = tgc_car_kine_wheel(PxF, RxF, VxF, OMxF, R0)
    
        # Transformation en dictionnaire pour une utilisation plus simple =====
        arg = ['pen','rz','angslip','angcamb','slip','Pct','Vmct','Rt_ground','dxF']
        tgc_dic = {}
        for i in range(len(tgc)):
            tgc_dic[arg[i]] = tgc[i] 
        
        # Force normale =======================================================
        if(tgc_dic['pen'] > 0):
            K = mbs_data.user_model['roue']['K']
            D = mbs_data.user_model['roue']['D']
            dedt = np.dot(RxF,tgc_dic['Vmct'])
            F[3] = K*tgc_dic['pen']-D*dedt[3]        
            
        # Force longitudinale =================================================
        # if (tgc_dic['pen'] >= 0):
        #     CFx = mbs_data.user_model['roue']['CFx']
        #     Flong = CFx*tgc_dic['slip']
        #     Flong_I = np.array([3., Flong, 0.,0.])
        #     Flong_Y = matrix_vector_product(RxF, Flong_I)
        #     for i in range(1,4):
        #         F[i] += Flong_Y[i]

        dxF = tgc_dic['dxF'][1:]
        
        verbose_tgc(analyseTGC,'../analyse/analyse_FWheel.txt',tgc_dic,ftype='txt')
        verbose_Force(analyseForce,'../analyse/Force_FWheel.txt' , F,ftype='txt')

    # Concatenating force, torque and force application point to returned array.
    # This must not be modified.
    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [F[1], F[2], F[3], M[1], M[2], M[3], dxF[0], dxF[1], dxF[2]]

    return Swr
