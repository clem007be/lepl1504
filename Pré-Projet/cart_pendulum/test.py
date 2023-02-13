# -*- coding: utf-8 -*-
"""
Created on Sun Feb 12 22:53:13 2023

@author: cleme
"""

import numpy as np
import matplotlib.pyplot as plt
import template_cartpendulum as cpdl

mbsdata = cpdl.MBSData()

ref = np.loadtxt('dirdyn_positions_ref.res')
teval = ref[:,0]
sol = cpdl.compute_dynamic_response(mbsdata)
sol = np.loadtxt('dirdyn_q.res')

# e = ref[:,1] - sol[:,1]

plt.figure()
plt.subplot(311)
plt.plot(sol[:,0],sol[:,1])
plt.scatter(ref[:,0],ref[:,1],c='red')
plt.subplot(312)
plt.plot(sol[:,0],sol[:,2])
plt.scatter(ref[:,0], ref[:,2],c='red')
# plt.subplot(313)
# plt.plot(teval, e,c='red')
plt.show()