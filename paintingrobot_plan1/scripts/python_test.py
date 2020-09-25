#!/usr/bin/env python
# -*- coding: utf-8 -*-
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import scipy.io
from collections import defaultdict

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

