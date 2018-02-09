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

chat=[None]
#counter=0
def callback_joint(msg):
  global chat
  #global counter

  chat.append(msg.data)
  #counter=counter+1
  
  
def main():
    # Create Pose Publisher and Subscriber
    rospy.Subscriber('/chatter', String, callback_joint)
    rospy.init_node('Talkity_talk_talk',anonymous=True)
    rate = rospy.Rate(100) 
    rospy.sleep(10)
    print('This is the Data: ', chat)

if __name__ == '__main__':
    try:
      main()
      
    except rospy.ROSInterruptException:
      
      pass