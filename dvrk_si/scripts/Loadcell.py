#!/usr/bin/env python
# http://wiki.ros.org/rospy/Overview/Messages

#rostopic pub /dvrk/PSM1/set_robot_state std_msgs/String DVRK_POSITION_CARTESIAN
# rostopic info /dvrk/MTML/set_position_joint

import rospy
import math
import time
import csv
from numpy import genfromtxt
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String

r=500 #frequency Hz of the publisher and subscriber
max_vel12=1
max_vel3=0.02
counter=0
torque_output2=[None]*100000000
timestamp=[None]*10000000

def callback_joint(msg):
  global joint_sub
  global counter
  joint_sub=msg
  
  #torque_output2[counter]=msg.effort
  #timestamp[counter]=msg.header.stamp.secs
  #timestamp[counter][2]=msg.header.stamp.nsecs
  
  counter=counter+1

def main(ra,max_vel12,max_vel3):
    # Create Pose Publisher and Subscriber
    state_pub=rospy.Publisher('/dvrk/PSM1/set_robot_state',String,queue_size=10)
    joint_pub= rospy.Publisher('/dvrk/PSM1/set_position_joint', JointState, queue_size=10)
    effort_pub= rospy.Publisher('/dvrk/PSM1/set_torque_joint', JointState, queue_size=10)
    rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, callback_joint)
    rospy.init_node('Talkity_talk_talk',anonymous=True)
    rate = rospy.Rate(ra) 
    rospy.sleep(1)
    
    #set robot state
    state_pub.publish("DVRK_POSITION_JOINT")
    rospy.sleep(1)

    #hardcode home to zero 
    joint_msg= JointState()
    joint_msg.header = Header()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer__wrist_pitch','outer_wrist_jaw', 'jaw']
    joint_msg.position=[0]*7
    joint_msg.velocity = []  
    joint_msg.effort = []
    
    rospy.sleep(1)

    vel_scale=1
    #d12=max_vel12/ra*scale
    #d3=max_vel3/ra*scale
    target1=-0.111338
    target3= 0.113058

    d=0.006
    thresh=0.008
    thresh3=thresh/5
    dq1=thresh+1
    dq2=thresh+1
    dq3=thresh+1
    while not rospy.is_shutdown():
    
      if -thresh<dq1-target1<thresh and -thresh<dq2<thresh  and -thresh<dq3-target3<thresh: 
        joint_msg.position=[]
        print(joint_msg)
        joint_pub.publish(joint_msg)
        rospy.sleep(1)
        break
      else:
        if -thresh<joint_sub.position[0]-target1<thresh:
          dq1=joint_sub.position[0] 
        elif joint_sub.position[0]-target1<0: 
         dq1=joint_sub.position[0]+d
        else: 
          dq1=joint_sub.position[0]-d

        if -thresh<joint_sub.position[1]<thresh: 
         dq2=joint_sub.position[1]
        elif joint_sub.position[1]<0: 
          dq2=joint_sub.position[1]+d
        else: 
          dq2=joint_sub.position[1]-d

        if -thresh3<joint_sub.position[2]-target3<3:
          dq3=joint_sub.position[2]
        elif joint_sub.position[2]-target3<0: 
          dq3=joint_sub.position[2]+d
        else: 
          dq3=joint_sub.position[2]-d
  
      #joint_msg.position = [dq1, dq2 , dq3 , None, None, None, None]
      joint_msg.position[0]=dq1
      joint_msg.position[1]=dq2
      joint_msg.position[2]=dq3
      joint_msg.position[3]=joint_sub.position[3]
      joint_msg.position[3]=joint_sub.position[4]
      joint_msg.position[3]=joint_sub.position[5]
      joint_msg.position[3]=joint_sub.position[6]

      print(joint_msg)
      joint_pub.publish(joint_msg)
      rospy.sleep(1/float(ra))

    #Effort Publisher
    rospy.sleep(1)
    state_pub.publish("DVRK_EFFORT_JOINT")
    rospy.sleep(2)

    effort_msg= JointState()
    effort_msg.header = Header()
    effort_msg.header.stamp = rospy.Time.now()
    #joint_msg.name = ['outer_yaw','o', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer__wrist_pitch','outer_wrist_jaw', 'jaw']
    effort_msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer__wrist_pitch','outer_wrist_jaw', 'jaw']
    effort_msg.position=[]
    effort_msg.velocity = []  
    effort_msg.effort = []

    #Excitation Trajectory Effort Values
    scale=1
    scale2=1
    i=0

    q_data3 = genfromtxt('T4.csv', delimiter=',')

    q_output=[None]*len(q_data3)
    qd_output=[None]*len(q_data3)
    torque_output=[None]*len(q_data3)
    timez=[None]*len(q_data3)
    while not rospy.is_shutdown():
      if i>len(q_data3)-10:
        effort_msg.effort=[]
        print(effort_msg)
        effort_pub.publish(effort_msg)
        break;
      effort_msg.position = []
      effort_msg.velocity = []  
      effort_msg.effort=[0,0,q_data3[i]*scale,0,0,0,0]
      print(effort_msg)
      effort_pub.publish(effort_msg)

      q_output[i]=joint_sub.position
      qd_output[i]=joint_sub.velocity
      torque_output[i]=joint_sub.effort
      timez[i]=[joint_sub.header.stamp.secs ,joint_sub.header.stamp.nsecs]
      time
      rospy.sleep(1/float(ra))
      i+=1
    rate.sleep()
    
    #Print Code
    with open('data_position.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(q_data3)-10):
          wr.writerow(q_output[i])
    with open('data_velocity.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(q_data3)-10):
          wr.writerow(qd_output[i])
    with open('data_torque.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(q_data3)-10):
          wr.writerow(torque_output[i])   
    with open('timestamp.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(q_data3)-10):
          wr.writerow(timez[i])   

#    with open('data_torque2.csv', 'wb') as myfile:
  #      wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
  #      for i in range(0,len(torque_output)-10):
  #        wr.writerow(torque_output[i])  
 #   with open('data_torque2.csv', 'wb') as myfile:
  #      wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
  #      for i in range(0,len(timestamp)-10):
  #        wr.writerow(timestamp[i])  


if __name__ == '__main__':
    try:
      main(r,max_vel12,max_vel3)
      
    except rospy.ROSInterruptException:

      pass
