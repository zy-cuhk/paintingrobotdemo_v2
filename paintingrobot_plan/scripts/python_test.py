#!/usr/bin/env python
# -*- coding: utf-8 -*-
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import scipy.io
from collections import defaultdict
import math

class multidict(dict):
    def __getitem__(self,item):
        try: 
            return dict.__getitem__(self,item)
        except KeyError: 
            value = self[item]=type(self)()
            return value

dict1=multidict()
dict1[0][0][0]=0.01
dict1[0][1]=2
print("dict1 is: ",dict1)

aubo_joints=np.array([0.6771608115099763, -1.9177900706742346, 0.7637025773712462, 2.681492648045481, 0.8936355152849202, 1.5707963267948966])*180/math.pi
print("aubo joints are: ",aubo_joints)

# "aubo_data_num_16": [
#     0.6771608115099763, 
#     -2.158228152120758, 
#     0.3874516362238376, 
#     -0.5959128652451966, 
#     -0.8936355152849202, 

    # 0.6771608115099754, 
    # -2.158228152120758, 
    # 0.3874516362238376, 
    # -0.5959128652451966, 
    # -0.8936355152849211, 
    # -1.5707963267948966

0.6771608115099745, -2.5296649265107756, -0.3874516362238367, -0.9993793633028538, -0.893635515284922, -1.5707963267948966
