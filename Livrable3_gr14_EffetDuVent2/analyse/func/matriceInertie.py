# -*- coding: utf-8 -*-
"""
Created on Thu May 11 21:10:16 2023

@author: cleme
"""

m = 32.5 
x = 1.32
y = 0.86
z = 1

Ixx = m/12*(y**2+z**2)
Iyy = m/12*(x**2+z**2)
Izz = m/12*(x**2+y**2)

print(Ixx)
print(Iyy)
print(Izz)