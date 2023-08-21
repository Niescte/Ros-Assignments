#!/usr/bin/python3

#On setting the goal_position, the turtlebot will move autonomously to that location
import rospy
import math
from math import pi, sqrt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def odom_callback(odom_msg):
    global current_x
    global current_y
    global current_angle
    current_x = odom_msg.pose.pose.position.x
    current_y = odom_msg.pose.pose.position.y
    orientation = odom_msg.pose.pose.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_angle = yaw * (180.0 / 3.141592653589793)
    print("x = ",current_x)
    print("y = ",current_y)
    print("Yaw:", current_angle)

rospy.init_node("pid")
rospy.Subscriber('/odom',Odometry,odom_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size = 1)
rospy.sleep(1)
goal_x = 0.0
goal_y = 0.0
rate = rospy.Rate(10)
#before this value
#distance pid coefficents
kp=2
kd=2
ki=1
#angle pid coefficents
kpa = 0.01
kda = 0.001
kia = 0.0001

twist = Twist()
distance = sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
total_distance = 0
diff_distance = 0
total_angle = 0
diff_angle = 0

while not rospy.is_shutdown():
    if(distance>0.1):
        
        #Tracking Distance
        distance = sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        total_distance = total_distance+distance
        velocity = kp*distance + kd*diff_distance + ki*total_distance
        if(velocity>0):
            twist.linear.x = min(0.22,velocity)
        else:
            twist.linear.x = max(-0.22,velocity)
        
        #Tracking Angle
        print("Target Angle = ",math.degrees(math.atan2(goal_y - current_y, goal_x - current_x)))
        angle = math.degrees(math.atan2(goal_y - current_y, goal_x - current_x))- current_angle
        if angle > 180:
            angle -= 360
        elif angle < -180:
            angle += 360
        total_angle = total_angle + angle
        ang_velocity = kpa*angle + kda*diff_angle + kia*total_angle
        if(ang_velocity>0):
            twist.angular.z = min(2.0,ang_velocity)
        else:
            twist.angular.z = max(-2.0,ang_velocity)
        
        #Publishing Velocities
        cmd_vel_pub.publish(twist)
        diff_distance = sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2) - distance
        diff_angle = (math.degrees(math.atan2(goal_y - current_y, goal_x - current_x))- current_angle) - angle
    else:
        twist.linear.x = 0
        twist.angular.z = 0
        cmd_vel_pub.publish(twist)
    rate.sleep()
