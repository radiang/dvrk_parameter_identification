#!/usr/bin/env python
from numpy import genfromtxt
import numpy as np
filename =  './test/6dof_31par_test1/Test.csv'
q= genfromtxt(filename, delimiter=',')

qt= q.transpose()

print(np.shape(q))
#print(np.max(q[0:18][3]))
print(q[0:6,[1]])

print(qt[[1],0:6])
print(qt[1][0:6])

#print(qt[0:6][3])