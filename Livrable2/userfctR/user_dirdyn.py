# -*- coding: utf-8 -*-
"""Module for the definition of functions related to Equilibrium analysis."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

from MBsysPy import MbsSensor

def user_dirdyn_init(mbs_data, mbs_dirdyn):
    """Run specific operation required by the user before running direct dynamic.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.

    """
    # mbs_data.sensors.append(MbsSensor(mbs_data))
    # mbs_data.sensors[-1].comp_s_sensor(mbs_data.sensor_id['Sensor_RWheel'])
    # mbs_data.sensors.append(MbsSensor(mbs_data))
    # mbs_data.sensors[-1].comp_s_sensor(mbs_data.sensor_id['Sensor_FWheel'])

    
    # Example: Creating and storing a sensor in mbs_data then create a list to store
    #          the vertical velocity of the sensor. Two new fields are added to
    #          the MbsData instance to store the sensor and the list.
    #
    # import MBsysPy as Robotran # Should be done outside the function.
    #
    # mbs_data.my_sensor = Robotran.MbsSensor(mbs_data)
    # mbs_data.my_sensor_v = []

    return


def user_dirdyn_loop(mbs_data, mbs_dirdyn):
    """Run specific operation required by the user at the end of each integrator step.

    In case of multistep integrator, this function is not called at intermediate
    steps.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.
    """
    sensor_RWheel = MbsSensor(mbs_data)
    sensor_FWheel = MbsSensor(mbs_data)
    sensor_RWheel.comp_s_sensor(mbs_data.sensor_id["Sensor_RWheel"]) 
    sensor_FWheel.comp_s_sensor(mbs_data.sensor_id["Sensor_FWheel"])
    
    mbs_data.set_output(sensor_RWheel.P[1], "Sensor_RWheelX")
    mbs_data.set_output(sensor_FWheel.P[1], "Sensor_FWheelX")
    mbs_data.set_output(sensor_RWheel.P[3], "Sensor_RWheelZ")
    mbs_data.set_output(sensor_FWheel.P[3], "Sensor_FWheelZ")
    

    # Example: The sensor, created in `user_dirdyn_init`, is computed (fields
    #          are updated) and the vertical velocity is added in the list.
    #
    # mbs_data.my_sensor.comp_s_sensor(1)
    # mbs_data.my_sensor_v.append(mbs_data.my_sensor.V[3])

    return


def user_dirdyn_finish(mbs_data, mbs_dirdyn):
    """Run specific operations required by the user when direct dynamic analysis ends.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.

    """

    # Example: The velocities are saved to a file, then the fields created in
    #          the MbsData instance during the function `user_dirdyn_init` are
    #          removed.
    #
    # import numpy as np # Should be done outside the function.
    #
    # np.savetxt("myfile.txt", mbs_data.my_sensor_v)
    # del mbs_data.my_sensor, mbs_data.my_sensor_v

    return