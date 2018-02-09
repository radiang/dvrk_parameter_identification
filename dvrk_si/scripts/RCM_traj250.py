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

r=250 #frequency Hz of the publisher and subscriber
max_vel12=1
max_vel3=0.05

def callback_joint(msg):
  global joint_sub
  joint_sub=msg

def main(ra,max_vel12,max_vel3):
    # Create Pose Publisher and Subscriber
    state_pub=rospy.Publisher('/dvrk/PSM1/set_robot_state',String,queue_size=10)
    joint_pub= rospy.Publisher('/dvrk/PSM1/set_position_joint', JointState, queue_size=10)
    rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, callback_joint)
    rospy.init_node('Talkity_talk_talk',anonymous=True)
    rate = rospy.Rate(ra) # 1hz
    rospy.sleep(1)
    
    #set robot state
    state_pub.publish("DVRK_POSITION_JOINT")
    rospy.sleep(1)

  
    joint_msg= JointState()
    joint_msg.header = Header()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer__wrist_pitch','outer_wrist_jaw', 'jaw']
    joint_msg.position=[0]*7
    joint_msg.velocity = []  
    joint_msg.effort = []
    rospy.sleep(1)

    
    #Go to negative q2 position
    d12=0.02
    d3=0.005/10 
    thresh=0.01

    target=-0.6

    dq1=thresh+1
    dq2=thresh+1
    dq3=thresh+1
    while not rospy.is_shutdown():
      if -thresh<dq1<thresh and -thresh<dq2-target<thresh  and -thresh<dq3<thresh: 
        break
      else:
        if -thresh<joint_sub.position[0]<thresh:
          dq1=joint_sub.position[0] 
        elif joint_sub.position[0]<0: 
         dq1=joint_sub.position[0]+d12
        else: 
          dq1=joint_sub.position[0]-d12

        if -thresh<joint_sub.position[1]-target<thresh: 
         dq2=joint_sub.position[1]
        elif joint_sub.position[1]-target<0: 
          dq2=joint_sub.position[1]+d12
        else: 
          dq2=joint_sub.position[1]-d12

        if -thresh<joint_sub.position[2]<thresh:
          dq3=joint_sub.position[2]
        elif joint_sub.position[2]<0: 
          dq3=joint_sub.position[2]+d3
        else: 
          dq3=joint_sub.position[2]-d3
  

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

    rospy.sleep(3)
    
    #Go to positive q2 pos
    target=0.7

    dq1=thresh+1
    dq2=thresh+1
    dq3=thresh+1
    while not rospy.is_shutdown():
      if -thresh<dq1<thresh and -thresh<dq2-target<thresh  and -thresh<dq3<thresh: 
        break
      else:
        if -thresh<joint_sub.position[0]<thresh:
          dq1=joint_sub.position[0] 
        elif joint_sub.position[0]<0: 
         dq1=joint_sub.position[0]+d12
        else: 
          dq1=joint_sub.position[0]-d12

        if -thresh<joint_sub.position[1]-target<thresh: 
         dq2=joint_sub.position[1]
        elif joint_sub.position[1]-target<0: 
          dq2=joint_sub.position[1]+d12
        else: 
          dq2=joint_sub.position[1]-d12

        if -thresh<joint_sub.position[2]<thresh:
          dq3=joint_sub.position[2]
        elif joint_sub.position[2]<0: 
          dq3=joint_sub.position[2]+d3
        else: 
          dq3=joint_sub.position[2]-d3
  

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

    rospy.sleep(3)

    #Go to negative q1 pos

    target=-1

    dq1=thresh+1
    dq2=thresh+1
    dq3=thresh+1
    while not rospy.is_shutdown():
      if -thresh<dq1-target<thresh and -thresh<dq2<thresh  and -thresh<dq3<thresh: 
        break
      else:
        if -thresh<joint_sub.position[0]-target<thresh:
          dq1=joint_sub.position[0] 
        elif joint_sub.position[0]-target<0: 
         dq1=joint_sub.position[0]+d12
        else: 
          dq1=joint_sub.position[0]-d12

        if -thresh<joint_sub.position[1]<thresh: 
         dq2=joint_sub.position[1]
        elif joint_sub.position[1]<0: 
          dq2=joint_sub.position[1]+d12
        else: 
          dq2=joint_sub.position[1]-d12

        if -thresh<joint_sub.position[2]<thresh:
          dq3=joint_sub.position[2]
        elif joint_sub.position[2]<0: 
          dq3=joint_sub.position[2]+d3
        else: 
          dq3=joint_sub.position[2]-d3
      
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

    rospy.sleep(3)

    #Go to positive q1 pos

    target=1

    dq1=thresh+1
    dq2=thresh+1
    dq3=thresh+1
    while not rospy.is_shutdown():
      if -thresh<dq1-target<thresh and -thresh<dq2<thresh  and -thresh<dq3<thresh: 
        break
      else:
        if -thresh<joint_sub.position[0]-target<thresh:
          dq1=joint_sub.position[0] 
        elif joint_sub.position[0]-target<0: 
         dq1=joint_sub.position[0]+d12
        else: 
          dq1=joint_sub.position[0]-d12

        if -thresh<joint_sub.position[1]<thresh: 
         dq2=joint_sub.position[1]
        elif joint_sub.position[1]<0: 
          dq2=joint_sub.position[1]+d12
        else: 
          dq2=joint_sub.position[1]-d12

        if -thresh<joint_sub.position[2]<thresh:
          dq3=joint_sub.position[2]
        elif joint_sub.position[2]<0: 
          dq3=joint_sub.position[2]+d3
        else: 
          dq3=joint_sub.position[2]-d3
      
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

    rospy.sleep(3)





    #Print Code
#    with open('data_position.csv', 'wb') as myfile:
#        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
#        for i in range(0,len(q_data1)):
#          wr.writerow(q_output[i])
#    with open('data_velocity.csv', 'wb') as myfile:
#        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
#        for i in range(0,len(q_data1)):
#          wr.writerow(qd_output[i])
#    with open('data_torque.csv', 'wb') as myfile:
##        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
#        for i in range(0,len(q_data1)):
#          wr.writerow(torque_output[i])    
    

if __name__ == '__main__':
    try:
      main(r,max_vel12,max_vel3)
      
    except rospy.ROSInterruptException:
      
      pass
