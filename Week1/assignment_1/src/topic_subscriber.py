#!/usr/bin/python3
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

def callback(msg):
    if((msg.data)%2!=0):
        pub.publish("odd")
    else:
        pub.publish("even")

rospy.init_node("oddeven_classifier")
sub = rospy.Subscriber('integers',Int32,callback)
pub = rospy.Publisher('oddeven',String,queue_size = 10)
rospy.spin()


