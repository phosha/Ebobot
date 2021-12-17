#!/usr/bin/env python

import math

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String
from std_msgs.msg import Int32

r_ticks = 0
l_ticks = 0

def r_wheel_callback(data):
    global r_ticks
    r_ticks = data.data
    #rospy.loginfo("I heard %s ticks from r_wheel",data.data)

def l_wheel_callback(data):
    global l_ticks
    l_ticks = data.data
    #rospy.loginfo("I heard %s ticks from l_wheel",data.data)

rospy.init_node('odom_node')

rospy.Subscriber("r_wheel_node", Int32, r_wheel_callback)
rospy.Subscriber("l_wheel_node", Int32, l_wheel_callback)


odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0
y = 0
th = 0

vx = 0.1
vy = -0.1
vth = 0.1

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(100.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    dt = current_time - last_time

    rospy.loginfo(r_ticks-l_ticks)

    last_time = current_time
    r.sleep()



