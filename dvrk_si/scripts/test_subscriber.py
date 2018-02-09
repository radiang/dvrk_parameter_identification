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
  
  torque_output2[counter]=msg.effort
  timestamp[counter]=msg.header.stamp.secs
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
    start=joint_sub.position
    rospy.sleep(1)

    vel_scale=1
    #d12=max_vel12/ra*scale
    #d3=max_vel3/ra*scale
    target1=-0.111338
    target2=0.11
    target3= 0.113058

    #target1=0
    #target2=0
    #target3=0
    
    d=0.0001
    thresh=0.008
    thresh3=thresh/5
    dq1=start[0]
    dq2=start[1]
    dq3=start[2]
    while not rospy.is_shutdown():
    
      if -thresh<dq1-target1<thresh and -thresh<dq2-target2<thresh  and -thresh<dq3-target3<thresh: 
        joint_msg.position=[]
        print(joint_msg)
        joint_pub.publish(joint_msg)
        rospy.sleep(1)
        break
      else:
        if -thresh<dq1-target1<thresh:
          dq1=dq1
        elif start[0]-target1<0: 
         dq1=dq1+d
        else: 
          dq1=dq1-d

        if -thresh<dq2-target2<thresh: 
         dq2=dq2    
        elif dq2-target2<0: 
          dq2=dq2+d
        else: 
          dq2=dq2-d

        if -thresh3<dq3-target3<thresh:
          dq3=dq3
        elif dq3-target3<0: 
          dq3=dq3+d
        else: 
          dq3=dq3-d
  
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

     # with open('timestamp.csv', 'wb') as myfile:
     #   wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
     #   for i in range(0,len(timestamp)-10):
     #     wr.writerow(timestamp[i])  


if __name__ == '__main__':
    try:
      main(r,max_vel12,max_vel3)
      
    except rospy.ROSInterruptException:
      
      pass
