#!/usr/bin/env python
from numpy import genfromtxt
import rospy
import math
import time
import csv
import numpy as np
import dvrk
import matplotlib.pyplot as plt
import datetime

foldername = './data/stribeck_3dof_svd/'
testname =  'brtest'

q  = genfromtxt(foldername+testname+'.csv', delimiter=',')
qt = q.transpose()

times = 1
speedscale=0.5
scale = 1

if times ==1: 
	a = q;
else:	
	a = np.zeros((len(q[:][1]),len(q[1][:])*times))

	#Make twice the trajectory
	for i in range(len(qt[1][:])):
		a[i][:]=np.append(q[i][:],q[i][:])

p=dvrk.psm('PSM1')
r=rospy.Rate(200*speedscale)
p.home()

period_data = len(a[0][:])
data_cycle = len(a)


states = np.zeros((len(a)*len(a[1][:]),3*3))

j = 0
while j<data_cycle and not rospy.is_shutdown():

	p.move_joint_some(np.array([0, 0, scale*a[j][0], 0, 0, 0]),np.array([0,1,2,3,4,5]))

	#p.move_joint_some(np.array([scale*a[0][0], scale*a[1][0], scale*a[2][0]]),np.array([0,1,2]))
	i = 0

	while  i<period_data and not rospy.is_shutdown():

		if math.isnan(a[j][i]):
			states[j*period_data+i][0:3] = [0, 0, 0]
			states[j*period_data+i][3:6] = [0, 0, 0]
			states[j*period_data+i][6:9] = [0, 0, 0]

		else:
			
			p.set_effort_joint(np.array([0,0,scale*a[j][i],0,0,0]))
			states[j*period_data+i][0:3] = p.get_current_joint_position()[0:3]
			states[j*period_data+i][3:6] = p.get_current_joint_velocity()[0:3]
			states[j*period_data+i][6:9] = p.get_current_joint_effort()[0:3]

		#print(states[(j)*period_data+i][0:3])
		print((j)*period_data+i,math.isnan(a[j][i]))		
		r.sleep()
		i = i +1

	j = j +1

#plt.show()

with open(foldername+testname+'_results.csv', 'wb') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
    for k in range(np.size(states,0)):
      wr.writerow(states[k][:])


