#!/usr/bin/env python

#Code to start system identification of robot
import dvrk
import rospy
import rospkg
import numpy as np
from numpy import genfromtxt
def main():
	p=dvrk.psm('PSM1')

	rate = rospy.Rate(250) 
	p.home()
	
	#Home robot to zero 
	p.dmove_joint(np.array([0.0, 0.0, -0.05, 0.0, 0.0, 0.0, 0.0]))
	
	
	#parameters of the identification task 
	rospack = rospkg.RosPack()
	
	fileloc = rospack.get_path(dvrk_si) + '/test/6dof_31par_test1'
	filename = fileloc +'Test.csv'
	q= genfromtxt(filename, delimiter=',')
	qt= q.transpose()

 	q_output=[None]*len(q[1][:])
    qd_output=[None]*len(q[1][:])
    torque_output=[None]*len(q[1][:])
	
	for i in range(len(q1))
		p.move_joint_some(qt[i][0:6],np.array([0,1,2,3,4,5]))
		rate.sleep()

      	q_output[i][0:6]=dvrk.get_current_joint_position()
      	qd_output[i][0:6]=dvrk.get_current_joint_velocity()
      	torque_output[i][0:6]=get_current_joint_effort()

	#while not rospy.is_shutdown():
	#p.move_joint_one(0.2,0)

	    #Print Code

	pname = fileloc +'data_position.csv'
	vname = fileloc +'data_velocity.csv'
	ename = fileloc +'data_effort.csv'

	
    with open(pname, 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(q_data1)-10):
          wr.writerow(q_output[i])
    with open(vname, 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(q_data1)-10):
          wr.writerow(qd_output[i])
    with open(ename, 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(q_data1)-10):
          wr.writerow(torque_output[i])   


if __name__ == '__main__':
    try:
      main()
      
    except rospy.ROSInterruptException:
      
      pass
