#!/usr/bin/python3

#This code moves the bot in a straight line until it reaches within 0.2 m of a goal point.
import rospy
from math import sqrt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def odom_callback(odom_msg):
    global current_x
    global current_y
    current_x = odom_msg.pose.pose.position.x
    current_y = odom_msg.pose.pose.position.y

rospy.init_node('get_current_position_node')
rospy.Subscriber('/odom', Odometry, odom_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size =1)
rospy.sleep(1.0)
twist = Twist()
goal_x = 3.0
goal_y = 0.0
driving_forward = False
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    dist = sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
    if(dist>0.2):
        driving_forward = True
    else:
        driving_forward = False
    if driving_forward == True:
        twist.linear.x = 0.16
    else:
        twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    rate.sleep()


