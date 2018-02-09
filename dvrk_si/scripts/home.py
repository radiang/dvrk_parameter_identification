#!/usr/bin/env python
# http://wiki.ros.org/rospy/Overview/Messages

#rostopic pub /dvrk/PSM1/set_robot_state std_msgs/String DVRK_POSITION_CARTESIAN
# rostopic info /dvrk/MTML/set_position_joint

import rospy
import math
import time
import csv
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String

r=25 #frequency Hz of the publisher and subscriber
max_vel12=1
max_vel3=0.02

def callback_joint(msg):
  global joint_sub
  joint_sub=msg

def main(ra,max_vel12,max_vel3):
    # Create Pose Publisher and Subscriber
    state_pub=rospy.Publisher('/dvrk/PSM1/set_robot_state',String,queue_size=10)
    joint_pub= rospy.Publisher('/dvrk/PSM1/set_position_joint', JointState, queue_size=10)
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

    scale=0.6
    #d12=max_vel12/ra*scale
    #d3=max_vel3/ra*scale

    d=0.008
    thresh=0.008
    dq1=thresh+1
    dq2=thresh+1
    dq3=thresh+1
    while not rospy.is_shutdown():
    
      if -thresh<dq1<thresh and -thresh<dq2<thresh  and -thresh<dq3<thresh: 
      	joint_msg.position=[]
      	print(joint_msg)
      	joint_pub.publish(joint_msg)
        break
      else:
        if -thresh<joint_sub.position[0]<thresh:
          dq1=joint_sub.position[0] 
        elif joint_sub.position[0]<0: 
         dq1=joint_sub.position[0]+d
        else: 
          dq1=joint_sub.position[0]-d

        if -thresh<joint_sub.position[1]<thresh: 
         dq2=joint_sub.position[1]
        elif joint_sub.position[1]<0: 
          dq2=joint_sub.position[1]+d
        else: 
          dq2=joint_sub.position[1]-d

        if -thresh<joint_sub.position[2]<thresh:
          dq3=joint_sub.position[2]
        elif joint_sub.position[2]<0: 
          dq3=joint_sub.position[2]+d/5
        else: 
          dq3=joint_sub.position[2]-d/5
  

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


if __name__ == '__main__':
    try:
      main(r,max_vel12,max_vel3)
      
    except rospy.ROSInterruptException:
      
      pass
