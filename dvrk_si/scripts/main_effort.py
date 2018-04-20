#!/usr/bin/env python
from numpy import genfromtxt
import numpy as np
import dvrk


foldername = './data/3dof_inplanepitch_svd_traj/'
testname =  'PID_data.csv'

q= genfromtxt(foldername+testname, delimiter=',')
qt= q.transpose()

p=dvrk.psm('PSM2')
r=rospy.Rate(hz)
p.home()
p.move_joint_some(np.array([0.0,0.0,0.0]),np.array([0,1,2]))

data = np.zeroes(len(states[6][:]),3*3)

for i in range(len(states[6][:])):
	p.set_effort_joint(np.array([q[0][i], q[1][i], q[2][i], 0.0, 0.0, 0.0, 0.0]))
	
	data[i][0:3] = p.get_current_joint_position()[0:3]
	data[i][3:6] = p.get_current_joint_velocity()[0:3]
	data[i][6:9] = p.get_current_joint_effort()[0:3]

	r.Rate
#Print Code
with open(foldername+'effort_data.csv', 'wb') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
    for i in range(np.size(data,0)-10):
      wr.writerow(data[i])
