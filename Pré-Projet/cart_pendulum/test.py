# -*- coding: utf-8 -*-
"""
Created on Sun Feb 12 22:53:13 2023

@author: cleme
"""

import numpy as np
import matplotlib.pyplot as plt
import template_cartpendulum as cpdl

mbsdata = cpdl.MBSData()

sol = cpdl.compute_dynamic_response(mbsdata)
ref = np.loadtxt('dirdyn_positions_ref.res')

plt.figure()
plt.subplot(211)
plt.plot(sol.t,sol.y[0])
plt.scatter(ref[:,0],ref[:,1],c='red')
plt.subplot(212)
plt.plot(sol.t,sol.y[1])
plt.scatter(ref[:,0], ref[:,2],c='red')
plt.show()