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

class Node Wrapper():
	def __init__(self):
		x = 0



