#!/usr/bin/env python
from numpy import genfromtxt
import numpy as np
import math
import csv

foldername = './data/stribeck_3dof_svd/'
testname =  'frtest'

q= genfromtxt(foldername + testname + '.csv', delimiter=',')

#qn=np.array(q)
qt= q.transpose()
print(len(q[0][:]))
print(len(q))


print(q[10][20])
print(q[10][239])

print(math.isnan(q[10][20]))
print(math.isnan(q[10][239]))
#print(np.max(q[0:18][3]))

#print(qt[0:6][3])
# a = np.array([1,2,3,4,5,6,7,8,9])
# print(a[0:3])
# print(a[3:6])
# print(a[6:9])

# print(a[6])

# print(q[1][2000])

with open(foldername+'data.csv', 'wb') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
    for i in range(np.size(qt,0)-10):
      wr.writerow(qt[i])