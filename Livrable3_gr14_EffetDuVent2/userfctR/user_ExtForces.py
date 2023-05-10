# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019
from mbs_tgc import *
import numpy as np
import csv
from MBsysPy import MbsSensor
from MBsysPy import matrix_vector_product
import matplotlib.pyplot as plt

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
                
                
def extForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
#     Fx = 0.0
#     Fy = 0.0
#     Fz = 0.0
#     Mx = 0.0
#     My = 0.0
#     Mz = 0.0
#     Fn = 0.0
#     idpt = mbs_data.xfidpt[ixF]
#     dxF = mbs_data.dpt[1:, idpt]

#     F = np.array([3.,0.,0.,0.])
#     M = np.array([3.,0.,0.,0.])

#     Fext = mbs_data.extforce_id["ExtForce_RWheel"]
#     if ixF == Fext:
#         tgc = tgc_car_kine_wheel(PxF,RxF,VxF,OMxF,mbs_data.user_model['Wheel']['RayonNominal'])
#         pen = tgc[0]
#         rz = tgc[1]
#         angslip = tgc[2]
#         angcamb = tgc[3]
#         slip = tgc[4]
#         Pct = tgc[5]
#         Vmct = tgc[6]
#         Rt_ground = tgc[7]
#         dxF = tgc[8][1:]

#         if pen >= 0:
#             Fn = mbs_data.user_model['Wheel']['Kp']*pen - mbs_data.user_model['Wheel']['Dp']*Vmct[3]
#             Fz = Fn 
#             C_Mz = mbs_data.user_model['Wheel']['CoefRoueArriere']*mbs_data.user_model['Wheel']['C_Fy']*Fn
#             Fx = mbs_data.user_model['Wheel']['C_Fx']*slip
#             if (VxF[1]**2+VxF[2]**2) > 0.0001:
#                 Fy = -mbs_data.user_model['Wheel']['C_Fy']*angslip*Fn-C_Mz*OMxF[3]/(VxF[1]**2+VxF[2]**2)**0.5-mbs_data.user_model['Wheel']['C_phi']*angcamb*Fn
#             Mz = angslip*C_Mz
            

#         else :
#             if mbs_data.process == 2 :
#                 Fz = mbs_data.user_model['Wheel']['Kp']*pen - mbs_data.user_model['Wheel']['Dp']*Vmct[3]
            
#         Fx_T = np.array([3.,Fx, Fy, Fz])
#         M_T = np.array([3., Mx, My, Mz])
#         dxF_T = np.array([3., dxF[0], dxF[1], dxF[2]])

#         F_I = matrix_vector_product(Rt_ground, Fx_T)
#         M_I = matrix_vector_product(Rt_ground, M_T)
    
    
#         for i in range(1,4):
#             F[i] = F_I[i]
#             M[i] = M_I[i]
        


            



#     Fext = mbs_data.extforce_id["ExtForce_FWheel"]
#     if ixF == Fext:
#         tgc = tgc_car_kine_wheel(PxF,RxF,VxF,OMxF,mbs_data.user_model['Wheel']['RayonNominal'])
#         pen = tgc[0]
#         rz = tgc[1]
#         angslip = tgc[2]
#         angcamb = tgc[3]
#         slip = tgc[4]
#         Pct = tgc[5]
#         Vmct = tgc[6]
#         Rt_ground = tgc[7]
#         dxF = tgc[8][1:]
#         if pen >= 0 :
#             Fn = mbs_data.user_model['Wheel']['Kp']*pen - mbs_data.user_model['Wheel']['Dp']*Vmct[3]
#             Fz = Fn  
#             C_Mz = mbs_data.user_model['Wheel']['CoefRoueAvant']*mbs_data.user_model['Wheel']['C_Fy']*Fn
#             Fx = mbs_data.user_model['Wheel']['C_Fx']*slip
#             if (VxF[1]**2+VxF[2]**2) > 0.0001:
#                 Fy = -mbs_data.user_model['Wheel']['C_Fy']*angslip*Fn-C_Mz*OMxF[3]/(VxF[1]**2+VxF[2]**2)**0.5-mbs_data.user_model['Wheel']['C_phi']*angcamb*Fn
#             Mz = angslip*C_Mz

#         else :
#             if mbs_data.process == 2 :
#                 Fz = mbs_data.user_model['Wheel']['Kp']*pen - mbs_data.user_model['Wheel']['Dp']*Vmct[3]
        
            
#         Fx_T = np.array([3., Fx, Fy, Fz])
#         M_T = np.array([3., Mx, My, Mz])
#         dxF_T = np.array([3., dxF[0], dxF[1], dxF[2]])
        

#         F_I = matrix_vector_product(Rt_ground, Fx_T)
#         M_I = matrix_vector_product(Rt_ground, M_T)

#         for i in range(1,4):
#             F[i] = F_I[i]
#             M[i] = M_I[i]

#         #if F_I[1] >= F_I[3]*mbs_data.user_model['Wheel']['CoefRoueAvant']: 
#             #F_I[1] = F_I[3]*mbs_data.user_model['Wheel']['CoefRoueAvant']
#         #if F_I[2] >= F_I[3]*mbs_data.user_model['Wheel']['CoefRoueAvant']: 
#             #F_I[2] = F_I[3]*mbs_data.user_model['Wheel']['CoefRoueAvant']
        
#         #print(F_I[3]*mbs_data.user_model['Wheel']['CoefRoueAvant'])
        
        
#             #[   0.         5521.96828096 -562.74955457 4213.18486442]
#             #[   0.         5519.34799028 -558.63063338 4209.63131213]
#             #[   0.         5519.34799028 -558.63063338 4209.63131213]

#             #[ 0.00000000e+00 -8.88006037e+01 -2.04527988e-01  8.05474123e+02]
#             #[ 0.00000000e+00 -8.87852842e+01 -2.04532479e-01  8.05508567e+02]
#             #[ 0.00000000e+00 -8.87852842e+01 -2.04532479e-01  8.05508567e+02]

                
#     Fext = mbs_data.extforce_id["ExtForce_RoueAvant"]
#     if ixF == Fext:
#         tgc = tgc_car_kine_wheel(PxF,RxF,VxF,OMxF,mbs_data.user_model['Roue']['RayonNominal'])
#         pen = tgc[0]
#         rz = tgc[1]
#         angslip = tgc[2]
#         angcamb = tgc[3]
#         slip = tgc[4]
#         Pct = tgc[5]
#         Vmct = tgc[6]
#         Rt_ground = tgc[7]
#         dxF = tgc[8][1:]

#         if pen >= 0 :
#             Fn = mbs_data.user_model['Roue']['Kp']*pen - mbs_data.user_model['Roue']['Dp']*Vmct[3]
#             Fz = Fn 
#             C_Mz = mbs_data.user_model['Roue']['CoefRoue']*mbs_data.user_model['Roue']['C_Fy']*Fn
#             Fx = mbs_data.user_model['Roue']['C_Fx']*slip
#             if (VxF[1]**2+VxF[2]**2) > 0.0001:
#                 Fy = -mbs_data.user_model['Roue']['C_Fy']*angslip*Fn-C_Mz*OMxF[3]/(VxF[1]**2+VxF[2]**2)**0.5-mbs_data.user_model['Roue']['C_phi']*angcamb*Fn
#             Mz = angslip*C_Mz
#             print(Mz, " à tsim =", tsim)

#         else :
#             if mbs_data.process == 2 :
#                 Fz = mbs_data.user_model['Wheel']['Kp']*pen - mbs_data.user_model['Wheel']['Dp']*Vmct[3]
            
#         Fx_T = np.array([3.,Fx, Fy, Fz])
#         M_T = np.array([3., Mx, My, Mz])
#         dxF_T = np.array([3., dxF[0], dxF[1], dxF[2]])

#         F_I = matrix_vector_product(Rt_ground, Fx_T)
#         M_I = matrix_vector_product(Rt_ground, M_T)
        
#         for i in range(1,4):
#             F[i] = F_I[i]
#             M[i] = M_I[i] 
        

#     Fext = mbs_data.extforce_id["ExtForce_RoueDroite"]
#     if ixF == Fext:
#         tgc = tgc_car_kine_wheel(PxF,RxF,VxF,OMxF,mbs_data.user_model['Roue']['RayonNominal'])
#         pen = tgc[0]
#         rz = tgc[1]
#         angslip = tgc[2]
#         angcamb = tgc[3]
#         slip = tgc[4]
#         Pct = tgc[5]
#         Vmct = tgc[6]
#         Rt_ground = tgc[7]
#         dxF = tgc[8][1:]

#         if pen >= 0 :
#             Fn = mbs_data.user_model['Roue']['Kp']*pen - mbs_data.user_model['Roue']['Dp']*Vmct[3]
#             Fz = Fn 
#             C_Mz = mbs_data.user_model['Roue']['CoefRoue']*mbs_data.user_model['Roue']['C_Fy']*Fn
#             Fx = mbs_data.user_model['Roue']['C_Fx']*slip
#             if (VxF[1]**2+VxF[2]**2) > 0.0001:
#                 Fy = -mbs_data.user_model['Roue']['C_Fy']*angslip*Fn-C_Mz*OMxF[3]/(VxF[1]**2+VxF[2]**2)**0.5-mbs_data.user_model['Roue']['C_phi']*angcamb*Fn
#             Mz = angslip*C_Mz

#         else :
#             if mbs_data.process == 2 :
#                 Fz = mbs_data.user_model['Wheel']['Kp']*pen - mbs_data.user_model['Wheel']['Dp']*Vmct[3]
            
#         Fx_T = np.array([3.,Fx, Fy, Fz])
#         M_T = np.array([3., Mx, My, Mz])
#         dxF_T = np.array([3., dxF[0], dxF[1], dxF[2]])

#         F_I = matrix_vector_product(Rt_ground, Fx_T)
#         M_I = matrix_vector_product(Rt_ground, M_T)
        
#         for i in range(1,4):
#             F[i] = F_I[i]
#             M[i] = M_I[i] 
        

#     Fext = mbs_data.extforce_id["ExtForce_RoueGauche"]
#     if ixF == Fext:
#         tgc = tgc_car_kine_wheel(PxF,RxF,VxF,OMxF,mbs_data.user_model['Roue']['RayonNominal'])
#         pen = tgc[0]
#         rz = tgc[1]
#         angslip = tgc[2]
#         angcamb = tgc[3]
#         slip = tgc[4]
#         Pct = tgc[5]
#         Vmct = tgc[6]
#         Rt_ground = tgc[7]
#         dxF = tgc[8][1:]

#         if pen >= 0 :
#             Fn = mbs_data.user_model['Roue']['Kp']*pen - mbs_data.user_model['Roue']['Dp']*Vmct[3]
#             Fz = Fn 
#             C_Mz = mbs_data.user_model['Roue']['CoefRoue']*mbs_data.user_model['Roue']['C_Fy']*Fn
#             Fx = mbs_data.user_model['Roue']['C_Fx']*slip
#             if (VxF[1]**2+VxF[2]**2) > 0.01:
#                 Fy = -mbs_data.user_model['Roue']['C_Fy']*angslip*Fn-C_Mz*OMxF[3]/(VxF[1]**2+VxF[2]**2)**0.5-mbs_data.user_model['Roue']['C_phi']*angcamb*Fn
#             Mz = angslip*C_Mz

#         else :
#             if mbs_data.process == 2 :
#                 Fz = mbs_data.user_model['Wheel']['Kp']*pen - mbs_data.user_model['Wheel']['Dp']*Vmct[3]
            
#         Fx_T = np.array([3.,Fx, Fy, Fz])
#         M_T = np.array([3., Mx, My, Mz])
#         dxF_T = np.array([3., dxF[0], dxF[1], dxF[2]])

#         F_I = matrix_vector_product(Rt_ground, Fx_T)
#         M_I = matrix_vector_product(Rt_ground, M_T)
        
#         for i in range(1,4):
#             F[i] = F_I[i]
#             M[i] = M_I[i] 
            
#         """if mbs_data.process == 2 and (PxF[3] - mbs_data.user_model['Wheel']['RayonNominal'])<0:
#         Fz = mbs_data.user_model['Wheel']['Kp']*(PxF[3] - mbs_data.user_model['Wheel']['RayonNominal']) - mbs_data.user_model['Wheel']['Dp']*Vmct[3]
        
#         Fx_T = np.array([3.,Fx, Fy, Fz])
#         M_T = np.array([3., Mx, My, Mz])

#         F_I = matrix_vector_product(Rt_ground, Fx_T)
#         M_I = matrix_vector_product(Rt_ground, M_T)
    
#         for i in range(1,4):
#             F[i] = F_I[i]
#             M[i] = M_I[i]"""


        
#     Swr = mbs_data.SWr[ixF]
#     Swr[1:] = [F[1], F[2], F[3], M[1], M[2], M[3], dxF[0], dxF[1], dxF[2]]

    return 0#Swr
            
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

            # Force Longitudinale =============================================
            CFx = mbs_data.user_model['roue']['CFx']
            Flong = CFx*tgc_dic['slip']
            
            # Force Latérale et Moment d'autoalignement =======================
            CFy = mbs_data.user_model['roue']['CFy/FN'] * FN
            Cphi = mbs_data.user_model['roue']['Cphi/FN'] * FN
            CMz = CFy * mbs_data.user_model['roue']['coefRWheel']
            Mz = CMz * tgc_dic['angslip']
            V2 = VxF[1]**2+VxF[2]**2
            if (V2) > 0.0001:
                Flat = -CFy * tgc_dic['angslip'] - Cphi * tgc_dic['angcamb'] - CMz * OMxF[3]/np.sqrt(V2)
            
            # Changement de base T -> I =======================================
            F_T = [3.0, Flong, Flat, FN]
            M_T = [3.0, 0, 0, Mz]
            F_I = matrix_vector_product(R_T_I, F_T)
            M_I = matrix_vector_product(R_T_I, M_T)
            
            # Assignation des forces ==========================================
            for i in range(1,4):
                F[i] = F_I[i]
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

            # Force Longitudinale =============================================
            CFx = mbs_data.user_model['roue']['CFx']
            Flong = CFx*tgc_dic['slip']
            
            # Force Latérale et Moment d'autoalignement =======================
            CFy = mbs_data.user_model['roue']['CFy/FN'] * FN
            Cphi = mbs_data.user_model['roue']['Cphi/FN'] * FN
            CMz = CFy * mbs_data.user_model['roue']['coefFWheel']
            Mz = CMz * tgc_dic['angslip']
            V2 = VxF[1]**2 + VxF[2]**2
            if (V2) > 0.0001:
                Flat = -CFy * tgc_dic['angslip'] - Cphi * tgc_dic['angcamb'] - CMz * OMxF[3]/np.sqrt(V2)
            
            # Changement de base T -> I =======================================
            F_T = [3.0, Flong, Flat, FN]
            M_T = [3.0, 0, 0, Mz]
            F_I = matrix_vector_product(R_T_I, F_T)
            M_I = matrix_vector_product(R_T_I, M_T)
            
            # Assignation des forces ==========================================
            for i in range(1,4):
                F[i] = F_I[i]
                M[i] = M_I[i]
        
        verbose_tgc(analyseTGC, 'analyse_FWheel', tgc_dic, tsim, ftype='txt')
        verbose_Force(analyseForce, 'Force_FWheel', F, tsim, ftype='txt')
        
    # Contact Front Wheel Remorque ============================================
    id_Force = mbs_data.extforce_id['ExtForce_FWheel_Rem']
    if (ixF == id_Force):
        R0 = mbs_data.user_model['Roue_Rem']['R0']
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
            K = mbs_data.user_model['Roue_Rem']['K']
            D = mbs_data.user_model['Roue_Rem']['D']
            FN = K*tgc_dic['pen']-D*tgc_dic['Vmct'][3]

            # Force Longitudinale =============================================
            CFx = mbs_data.user_model['Roue_Rem']['CFx']
            Flong = CFx*tgc_dic['slip']
            
            # Force Latérale et Moment d'autoalignement =======================
            CFy = mbs_data.user_model['Roue_Rem']['CFy/FN'] * FN
            Cphi = mbs_data.user_model['Roue_Rem']['Cphi/FN'] * FN
            CMz = CFy * mbs_data.user_model['Roue_Rem']['coefRoue']
            Mz = CMz * tgc_dic['angslip']
            V2 = VxF[1]**2 + VxF[2]**2
            if (V2) > 0.0001:
                Flat = -CFy * tgc_dic['angslip'] - Cphi * tgc_dic['angcamb'] - CMz * OMxF[3]/np.sqrt(V2)
            
            # Changement de base T -> I =======================================
            F_T = [3.0, Flong, Flat, FN]
            M_T = [3.0, 0, 0, Mz]
            F_I = matrix_vector_product(R_T_I, F_T)
            M_I = matrix_vector_product(R_T_I, M_T)
            
            # Assignation des forces ==========================================
            for i in range(1,4):
                F[i] = F_I[i]
                M[i] = M_I[i]
                
        verbose_tgc(analyseTGC, 'analyse_FWheel_Rem', tgc_dic, tsim, ftype='txt')
        verbose_Force(analyseForce, 'Force_FWheel_Rem', F, tsim, ftype='txt')
                
    # Contact Right Wheel Remorque ============================================
    id_Force = mbs_data.extforce_id['ExtForce_RoueD']
    if (ixF == id_Force):
        
        R0 = mbs_data.user_model['Roue_Rem']['R0']
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
            K = mbs_data.user_model['Roue_Rem']['K']
            D = mbs_data.user_model['Roue_Rem']['D']
            FN = K*tgc_dic['pen']-D*tgc_dic['Vmct'][3]

            # Force Longitudinale =============================================
            CFx = mbs_data.user_model['Roue_Rem']['CFx']
            Flong = CFx*tgc_dic['slip']
            
            # Force Latérale et Moment d'autoalignement =======================
            CFy = mbs_data.user_model['Roue_Rem']['CFy/FN'] * FN
            Cphi = mbs_data.user_model['Roue_Rem']['Cphi/FN'] * FN
            CMz = CFy * mbs_data.user_model['Roue_Rem']['coefRoue']
            Mz = CMz * tgc_dic['angslip']
            V2 = VxF[1]**2 + VxF[2]**2
            if (V2) > 0.0001:                
                Flat = -CFy * tgc_dic['angslip'] - Cphi * tgc_dic['angcamb'] - CMz * OMxF[3]/np.sqrt(V2)

            
            # Changement de base T -> I =======================================
            F_T = [3.0, Flong, Flat, FN]
            M_T = [3.0, 0, 0, Mz]
            F_I = matrix_vector_product(R_T_I, F_T)
            M_I = matrix_vector_product(R_T_I, M_T)
            
            # Assignation des forces ==========================================
            for i in range(1,4):
                F[i] = F_I[i]
                M[i] = M_I[i]
                
        verbose_tgc(analyseTGC, 'analyse_RWheel_Rem', tgc_dic, tsim, ftype='txt')
        verbose_Force(analyseForce, 'Force_RWheel_Rem', F, tsim, ftype='txt')
        
    # Contact Left Wheel Remorque =============================================
    id_Force = mbs_data.extforce_id['ExtForce_RoueG']
    if (ixF == id_Force):
        
        R0 = mbs_data.user_model['Roue_Rem']['R0']
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
            K = mbs_data.user_model['Roue_Rem']['K']
            D = mbs_data.user_model['Roue_Rem']['D']
            FN = K*tgc_dic['pen']-D*tgc_dic['Vmct'][3]

            # Force Longitudinale =============================================
            CFx = mbs_data.user_model['Roue_Rem']['CFx']
            Flong = CFx*tgc_dic['slip']
            
            # Force Latérale et Moment d'autoalignement =======================
            CFy = mbs_data.user_model['Roue_Rem']['CFy/FN'] * FN
            Cphi = mbs_data.user_model['Roue_Rem']['Cphi/FN'] * FN
            CMz = CFy * mbs_data.user_model['Roue_Rem']['coefRoue']
            Mz = CMz * tgc_dic['angslip']
            V2 = VxF[1]**2 + VxF[2]**2
            if (V2) > 0.0001:
                Flat = -CFy * tgc_dic['angslip'] - Cphi * tgc_dic['angcamb'] - CMz * OMxF[3]/np.sqrt(V2)
            
            # Changement de base T -> I =======================================
            F_T = [3.0, Flong, Flat, FN]
            M_T = [3.0, 0, 0, Mz]
            F_I = matrix_vector_product(R_T_I, F_T)
            M_I = matrix_vector_product(R_T_I, M_T)
            
            # Assignation des forces ==========================================
            for i in range(1,4):
                F[i] = F_I[i]
                M[i] = M_I[i]  
            
        verbose_tgc(analyseTGC, 'analyse_LWheel_Rem', tgc_dic, tsim, ftype='txt')
        verbose_Force(analyseForce, 'Force_LWheel_Rem', F, tsim, ftype='txt')
        
    # Concatenating force, torque and force application point to returned array.
    # This must not be modified.
    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [F[1], F[2], F[3], M[1], M[2], M[3], dxF[0], dxF[1], dxF[2]]

    return Swr
