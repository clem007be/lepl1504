#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Script to run a direct dynamic analysis on a multibody system.

Summary
-------
This template loads the data file *.mbs and execute:
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - the equilibrium module
 - the modal module
 - the inverse dynamic module
 - the solverkin module

It may have to be adapted and completed by the user.


Universite catholique de Louvain
CEREM : Centre for research in mechatronics

http://www.robotran.eu
Contact : info@robotran.be

(c) Universite catholique de Louvain
"""

# %%===========================================================================
# Packages loading
# =============================================================================
try:
    import MBsysPy as Robotran
    import matplotlib.pyplot as plt
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/"
                      )

# %%===========================================================================
# Project loading
# =============================================================================
mbs_data = Robotran.MbsData("../dataR/Livrable1.mbs")



# %%===========================================================================
# Partitionning
# =============================================================================
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

#=============================================================================
#Equilibrium
#=============================================================================

mbs_data.process = 2
mbs_equil = Robotran.MbsEquil(mbs_data)
mbs_equil.set_options(method=1, senstol=1e-2, verbose=1)
results = mbs_equil.run()

# Figure creation
fig = plt.figure(num='equilibrium')
axis = fig.gca()

# Plotting data's
axis.plot(results.q[:, 0], results.q[:, 1], label='q[1]')

# Figure enhancement
axis.grid(True)
axis.set_xlim(left=mbs_equil.get_options('t0'), right=mbs_equil.get_options('tf'))
axis.set_xlabel('Time (s)')
axis.set_ylabel('Coordinate value (m or rad)')

plt.show()



# =============================================================================
# Modal Analysis
# =============================================================================
mbs_data.process = 4
mbs_modal = Robotran.MbsModal(mbs_data)
mbs_modal.set_options(save_result=1, save_anim=1, mode_ampl=0.2)
mbs_modal.run()

# # =============================================================================
# # Direct Dynamics
# # =============================================================================
# mbs_data.process = 3
# mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
# mbs_dirdyn.set_options(dt0=5e-4, tf=5, save2file=1)


# result = mbs_dirdyn.run()
# # Figure creation
# fig = plt.figure(num='dyrdin')
# axis = fig.gca()

# id_coord = 1
# # Plotting data's
# axis.plot(result.q[:, 0], result.q[:, id_coord], label='q[1]')

# # Figure enhancement
# axis.title("q",id_coord)
# axis.grid(True)
# axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))
# axis.set_xlabel('Time (s)')
# axis.set_ylabel('Coordinate value (m or rad)')

# plt.show()




"""
# =============================================================================
# Inverse Kinematics
# =============================================================================
mbs_data.process = 5
mbs_solvekin = Robotran.MbsSolvekin(mbs_data)
mbs_solvekin.set_options(trajectoryqname="../resultsR/dirdyn_q.res")
mbs_solvekin.set_options(trajectoryqdname="../resultsR/dirdyn_qd.res")
mbs_solvekin.set_options(trajectoryqddname="../resultsR/dirdyn_qdd.res")
mbs_solvekin.set_options(t0=1.3333, tf=0.2, dt=1e-4, framerate=10000)
mbs_solvekin.set_options(motion="trajectory")
mbs_solvekin.run()

# =============================================================================
# Inverse Dynamics
# =============================================================================
mbs_data.process = 6
mbs_invdyn = Robotran.MbsInvdyn(mbs_data)
mbs_invdyn.set_options(trajectoryqname="../resultsR/dirdyn_q.res")
mbs_invdyn.set_options(trajectoryqdname="../resultsR/dirdyn_qd.res")
mbs_invdyn.set_options(trajectoryqddname="../resultsR/dirdyn_qdd.res")
mbs_invdyn.set_options(t0=0.0, tf=0.2, dt=1e-3)
mbs_invdyn.set_options(motion="trajectory")
mbs_invdyn.run()
"""