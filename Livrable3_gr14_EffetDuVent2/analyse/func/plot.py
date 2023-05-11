# -*- coding: utf-8 -*-
"""
Created on Thu May 11 02:44:25 2023

@author: cleme
"""

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec

def binSearch(arr,prec):
    lo = 0
    hi = len(arr[0])-1
    mid = 0 
    while (lo <= hi):
        mid = (hi+lo)//2
        if (arr[0,mid]-5 > prec):
            lo = mid+1
        elif (arr[0,mid]-5 < 0):
            hi = mid-1
        else:
            return mid
    return -1
    
fig = plt.figure()
gs = gridspec.GridSpec(1, 1)
axis = fig.add_subplot(gs[0,0])
dic_v = {20: 0, 18: 0, 15: 0}#, 12: 0}
color_v = {20: 'blue', 18: 'orange', 15: 'red'}#, 12: 'green'}
dic_m = {0.01: 0, 32.5: 0, 130.0: 0}
color_m = {0.01: 'blue', 32.5: 'orange', 130.0: 'red'}

lines = []
for i in dic_m:
    dic_m[i] = np.loadtxt("../Demi-tour/v15/roulis0.3_m{}.txt".format(i))
    
    j = binSearch(dic_m[i],1e-2)
    dic_m[i] = dic_m[i][:,:j]
    
    line, = axis.plot(dic_m[i][1],dic_m[i][0], color=color_m[i])
    lines.append(line)
    
# j = dichotomicSearch(dic[12],1e-5)
# dic[12] = dic[12][:,:j]
# j = dichotomicSearch(dic[15],1e-5)
# dic[15] = dic[15][:,:j]
# j = dichotomicSearch(dic[18],1e-5)
# dic[18] = dic[18][:,:j]
# j = dichotomicSearch(dic[20],3e-3)
# dic[20] = dic[20][:,:j]    
    
axis.set_xlim(2,-27)
axis.set_ylim(5)
axis.grid(True)
plt.title('Trajectoire en fonction de la masse du chargement')
axis.set_aspect('equal','box')
axis.set_xlabel("y [m]")
axis.set_ylabel("x [m]")
axis.legend(lines, ['{} kg'.format(i) for i in color_m.keys()])

fig.savefig("../Demi-tour/graphe/varM_roulis0.3_v15.svg")
plt.show()


