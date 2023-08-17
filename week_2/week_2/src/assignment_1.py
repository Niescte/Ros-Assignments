#!/usr/bin/python3

#This code moves the bot with a constant x velocity

import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
rospy.init_node('velocity')
rospy.sleep(1)
twist = Twist()
twist.linear.x = 0.16
cmd_vel_pub.publish(twist)