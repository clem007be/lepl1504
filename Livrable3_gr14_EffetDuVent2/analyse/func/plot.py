# -*- coding: utf-8 -*-
"""
Created on Thu May 11 02:44:25 2023

@author: cleme
"""

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec


def binSearch(arr, prec):
    lo = 0
    hi = len(arr[0])-1
    mid = 0
    while (lo <= hi):
        mid = (hi+lo)//2
        if (arr[0, mid]-5 > prec):
            lo = mid+1
        elif (arr[0, mid]-5 < 0):
            hi = mid-1
        else:
            return mid
    return -1


dic_v = {20: 0, 18: 0, 15: 0, 12: 0}
color_v = {20: 'blue', 18: 'orange', 15: 'red', 12: 'green'}
dic_m = {0.01: 0, 32.5: 0}#, 130.0: 0}
color_m = {0.01: 'blue', 32.5: 'orange'}#, 130.0: 'red'}

lines = []

fig = plt.figure()
fig.set_tight_layout(True)
gs = gridspec.GridSpec(1, 2)
axis = fig.add_subplot(gs[0, 0])
for i in dic_v:
    dic_v[i] = np.loadtxt("../Evitement/v{}_m0.01.txt".format(i))

#     j = binSearch(dic_v[i],1e-2)
#     dic_v[i] = dic_v[i][:,:j]

    axis.plot(dic_v[i][1], dic_v[i][0], color=color_v[i])

axis.set_aspect('equal')
axis.grid(True)
axis.set_xlabel("y [m]")
axis.set_ylabel("x [m]")
axis.set_ylim(top=50)

axis = fig.add_subplot(gs[0, 1])
for i in dic_v:
    line, = axis.plot(dic_v[i][1], dic_v[i][0], color=color_v[i])
    lines.append(line)

axis.grid(True)
axis.set_xlabel("y [m]")
axis.set_ylabel("x [m]")
axis.set_ylim(top=50)
plt.title('Trajectoire en fonction de\n la masse du chargement')
axis.legend(lines, ['{} kg'.format(i) for i in color_v.keys()])

# j = dichotomicSearch(dic[12],1e-5)
# dic[12] = dic[12][:,:j]
# j = dichotomicSearch(dic[15],1e-5)
# dic[15] = dic[15][:,:j]
# j = dichotomicSearch(dic[18],1e-5)
# dic[18] = dic[18][:,:j]
# j = dichotomicSearch(dic[20],3e-3)
# dic[20] = dic[20][:,:j]
#

fig.savefig("../Evitement/graphe/varV_m0.01.svg")
plt.show()
