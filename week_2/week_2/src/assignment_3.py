#!/usr/bin/python3


#This code moves the turtlebot with a constant velocity and stops the bot 0.3 m before an obstacle. At 0.1 m turtlebot collides with obstacle
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    global obj_distance
    obj_distance = min(msg.ranges)
    print(obj_distance)

scan_sub = rospy.Subscriber('scan',LaserScan,scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
rospy.init_node('wander')
twist = Twist()
cmd_vel_pub.publish(twist)
rospy.sleep(1)

driving_forward = True
rate = rospy.Rate(10)
while not rospy.is_shutdown():
     if(obj_distance < 0.3):
          driving_forward = False
     else:
          driving_forward = True
     if (driving_forward == True):
          twist.linear.x = 0.16
          twist.angular.z = 0
     else:
          twist.linear.x = 0
          twist.angular.z = 0
     cmd_vel_pub.publish(twist)
     rate.sleep()
