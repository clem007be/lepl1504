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
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/"
                      )

# %%===========================================================================
# Project loading
# =============================================================================
K = np.linspace(1e4,1e5,101,endpoint=True)
# for i in K:
mbs_data = Robotran.MbsData("../dataR/Remorque1.mbs")

# %%===========================================================================
# Partitionning
# =============================================================================
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# # =============================================================================
# # Equilibrium
# # =============================================================================
# mbs_data.process = 2
# mbs_equil = Robotran.MbsEquil(mbs_data)
# mbs_equil.set_options(method=1, senstol=1e-2, verbose=1)
# mbs_equil.run()

# # =============================================================================
# # Modal Analysis
# # =============================================================================
# mbs_data.process = 4
# mbs_modal = Robotran.MbsModal(mbs_data)
# mbs_modal.set_options(save_result=1, save_anim=1, mode_ampl=0.2)
# mbs_modal.run()

# =============================================================================
# Direct Dynamics
# =============================================================================
mbs_data.process = 3
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
mbs_dirdyn.set_options(dt0=1e-3, tf=2.0, save2file=1)
results = mbs_dirdyn.run()

# =============================================================================
# Plotting
# =============================================================================

fig = plt.figure()
axis = fig.gca()

axis.plot(results.q[:,0], results.q[:,13])
plt.show()