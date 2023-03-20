#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Script to run a direct dynamic analysis on a multibody system.

Summary
-------
This template loads the data file *.mbs and execute:
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - if available, plot the time evolution of the first generalized coordinate.

It may have to be adapted and completed by the user.


Universite catholique de Louvain
CEREM : Centre for research in mechatronics

http://www.robotran.eu
Contact : info@robotran.be

(c) Universite catholique de Louvain
"""

# %%============================================================================
# Packages loading
# =============================================================================
try:
    import MBsysPy as Robotran
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/"
                      )

# %%===========================================================================
# Project loading
# =============================================================================
mbs_data = Robotran.MbsData('../dataR/Livrable2.mbs')

#cleaning file analyse.txt
try:
    f = open('../analyse/analyse_RWheel.txt','w')
    f.close()
    f = open('../analyse/analyse_FWheel.txt','w')
    f.close()
    f = open('../analyse/Force_FWheel.txt','w')
    f.close()
    f = open('../analyse/Force_RWheel.txt','w')
    f.close()
except:
    print('unable to open file')
# %%===========================================================================
# Partitionning
# =============================================================================
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# %%===========================================================================
# Direct Dynamics
# =============================================================================
mbs_data.process = 3
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
mbs_dirdyn.set_options(dt0=1e-3, tf=10.0, save2file=1)
results = mbs_dirdyn.run()

# %%===========================================================================
# Plotting results
# =============================================================================
try:
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec

except Exception:
    raise RuntimeError('Unable to load matplotlib, plotting results unavailable.')


# Figure creation
fig = plt.figure(num='Example of plot')
fig.clear()
fig.set_tight_layout(True)
gs = gridspec.GridSpec(3,4)

# Plotting data's
for i in range(1,len(results.q[0])):
# for i in [1,3,6,7,10]:
    axis = fig.add_subplot(gs[(i-1)//4, (i-1)%4])
    axis.plot(results.q[:, 0], results.q[:, i])
    
    axis.set_title('q[{}]'.format(i))
    axis.grid(True)
    axis.set_xlabel('Time (s)')
    axis.set_ylabel('Coordinate value (m or rad)')

# Figure enhancement
# axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))


# fig2 = plt.figure('Sensor')
# fig2.clear()
# gs = gridspec.GridSpec(3,2)

# axis = fig2.add_subplot(gs[0,0])
# axis.plot(results.q[:,0], results.q[:,10])
# axis.grid(True)
# axis.set_title('front wheel')
# axis = fig2.add_subplot(gs[1,0])
# axis.plot(results.qd[:,0], results.qd[:,10])
# axis.grid(True)
# axis = fig2.add_subplot(gs[2,0])
# axis.plot(results.qdd[:,0], results.qdd[:,10])
# axis.grid(True)
# axis = fig2.add_subplot(gs[0,1])
# axis.plot(results.q[:,0], results.q[:,7])
# axis.grid(True)
# axis.set_title('rear wheel')
# axis = fig2.add_subplot(gs[1,1])
# axis.plot(results.qd[:,0], results.qd[:,7])
# axis.grid(True)
# axis = fig2.add_subplot(gs[2,1])
# axis.plot(results.qdd[:,0], results.qdd[:,7])
# axis.grid(True)

plt.show()
