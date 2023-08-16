#!/usr/bin/python3

import rospy
import random

from std_msgs.msg import Int32

rospy.init_node('integer_generator')

pub = rospy.Publisher('integers',Int32,queue_size = 10)

rate = rospy.Rate(2)

while not rospy.is_shutdown():
	count = random.randint(1,500)
	pub.publish(count)
	rate.sleep()
