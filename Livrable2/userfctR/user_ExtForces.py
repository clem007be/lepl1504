# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019
from mbs_tgc import *
import numpy as np
import csv
from MBsysPy import MbsSensor
from MBsysPy import matrix_vector_product

def verbose_tgc(boo, filename, tgc_dic, tsim, ftype='txt'):
    if(boo):
        file = '../analyse/{}.{}'.format(filename,ftype)
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
                f.write('Time : {}\n'.format(tsim))
                for i in tgc_dic:    
                    f.write('{} : {}\n'.format(i, tgc_dic[i]))
                f.write('\n')
                f.close()
            except :
                print("unable to open the file tgc.txt") 
            
def verbose_Force(boo, filename, Force, tsim, ftype='txt'):
    if(boo):
        file = '../analyse/{}.{}'.format(filename,ftype)
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
                f.write('Time : {}\n'.format(tsim))
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
    
    analyseTGC = False
    analyseForce = False
    
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
            
        R_T_I = tgc_dic['Rt_ground']
        dxF = tgc_dic['dxF'][1:]
        
        if(tgc_dic['pen'] > 0):
            # Force Normale ===================================================
            K = mbs_data.user_model['roue']['K']
            D = mbs_data.user_model['roue']['D']
            FN = K*tgc_dic['pen']-D*tgc_dic['Vmct'][3]
            FN_T = np.array([3., 0., 0., FN])

            # Force Longitudinale =============================================
            CFx = mbs_data.user_model['roue']['CFx']
            Flong = CFx*tgc_dic['slip']
            Flong_T = np.array([3., Flong, 0., 0.])
            
            # Force Latérale et Moment d'autoalignement =======================
            CFy = mbs_data.user_model['roue']['CFy/FN'] * FN
            Cphi = mbs_data.user_model['roue']['Cphi/FN'] * FN
            CMz = CFy * mbs_data.user_model['roue']['coefRWheel']
            Mz = CMz * tgc_dic['angslip']
            VxF_T = matrix_vector_product(R_T_I, VxF)
            Flat = -CFy * tgc_dic['angslip'] - Cphi * tgc_dic['angcamb'] - CMz * OMxF[3]/VxF_T[1]
            Flat_T = np.array([3., 0., Flat, 0.])
            M_T = np.array([3., 0., 0., Mz])
            
            # Changement de base T -> I =======================================
            FN_I = matrix_vector_product(R_T_I.T, FN_T)
            Flong_I = matrix_vector_product(R_T_I.T, Flong_T)
            Flat_I = matrix_vector_product(R_T_I.T, Flat_T)
            M_I = matrix_vector_product(R_T_I.T, M_T)
            
            # Assignation des forces ==========================================
            for i in range(1,4):
                F[i] = FN_I[i] + Flong_I[i] + Flat_I[i]
                M[i] = M_I[i]  
        
        verbose_tgc(analyseTGC, 'analyse_RWheel', tgc_dic, tsim, ftype='txt')
        verbose_Force(analyseForce, 'Force_RWheel', F, tsim, ftype='txt')
    
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
        
        R_T_I = tgc_dic['Rt_ground']
        dxF = tgc_dic['dxF'][1:]
        
        if(tgc_dic['pen'] > 0):
            # Force Normale ===================================================
            K = mbs_data.user_model['roue']['K']
            D = mbs_data.user_model['roue']['D']
            FN = K*tgc_dic['pen']-D*tgc_dic['Vmct'][3]
            FN_T = np.array([3., 0., 0., FN])

            # Force Longitudinale =============================================
            CFx = mbs_data.user_model['roue']['CFx']
            Flong = CFx*tgc_dic['slip']
            Flong_T = np.array([3., Flong, 0., 0.])
            
            # Force Latérale et Moment d'autoalignement =======================
            CFy = mbs_data.user_model['roue']['CFy/FN'] * FN
            Cphi = mbs_data.user_model['roue']['Cphi/FN'] * FN
            CMz = CFy * mbs_data.user_model['roue']['coefFWheel']
            Mz = CMz * tgc_dic['angslip']
            VxF_T = matrix_vector_product(R_T_I, VxF)
            Flat = -CFy * tgc_dic['angslip'] - Cphi * tgc_dic['angcamb'] - CMz * OMxF[3]/VxF_T[1]
            Flat_T = np.array([3., 0., Flat, 0.])
            M_T = np.array([3., 0., 0., Mz])
            
            # Changement de base T -> I =======================================
            FN_I = matrix_vector_product(R_T_I.T, FN_T)
            Flong_I = matrix_vector_product(R_T_I.T, Flong_T)
            Flat_I = matrix_vector_product(R_T_I.T, Flat_T)
            M_I = matrix_vector_product(R_T_I.T, M_T)
            
            # Assignation des forces ==========================================
            for i in range(1,4):
                F[i] = FN_I[i] + Flong_I[i] + Flat_I[i]
                M[i] = M_I[i]
        
        verbose_tgc(analyseTGC, 'analyse_FWheel', tgc_dic, tsim, ftype='txt')
        verbose_Force(analyseForce, 'Force_FWheel', F, tsim, ftype='txt')
    
    # Moment dans le guidon pour garder le vélo droit =========================
    Fext = mbs_data.extforce_id["ExtForce_Guidon"]
    if ixF == Fext:
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
            phid = mbs_data.qd[R1_d]
            K_phi = 80
            K_phid = 125
            Mz = - K_phi * (phi - phi0) - K_phid * phid
            M_T = np.array([3., 0., 0., Mz])
            M_I = matrix_vector_product(RxF.T,M_T)
            for i in range(1,4):
                M[i] = M_I[i]  
        
    # Concatenating force, torque and force application point to returned array.
    # This must not be modified.
    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [F[1], F[2], F[3], M[1], M[2], M[3], dxF[0], dxF[1], dxF[2]]

    return Swr
