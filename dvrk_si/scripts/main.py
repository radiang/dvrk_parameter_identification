#!/usr/bin/env python
from numpy import genfromtxt
import numpy as np
import dvrk

foldername = './data/3dof_inplanepitch_svd_traj/'
testname =  'test_position'

q= genfromtxt(foldername+testname+'.csv', delimiter=',')
qt= q.transpose()

p=dvrk.psm('PSM2')
r=rospy.Rate(2/0.1*0.5)
p.home()
p.move_joint_some(np.array([0.0,0.0,0.0]),np.array([0,1,2]))

states = np.zeroes(len(q[1][:]),3*3)

for i in range(len(q[1][:])):
	p.move_joint_some(np.array([q[1][i], q[2][i], q[3][i]]),np.array([0,1,2]))
	states[i][0:3] = p.get_current_joint_position()[0:3]
	states[i][3:6] = p.get_current_joint_velocity()[0:3]
	states[i][6:9] = p.get_current_joint_effort()[0:3]
	r.Rate

with open(foldername+'PID_data.csv', 'wb') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
    for i in range(np.size(data,0)-10):
      wr.writerow(states[i])
