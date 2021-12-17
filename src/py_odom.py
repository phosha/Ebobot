#!/usr/bin/env python

import math

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32


wheel_c = 210
tpm = 1714.28
base_wight = 0.19


r_ticks = 0
l_ticks = 0
imu_ticks = 0

def r_wheel_callback(data):
    global r_ticks
    r_ticks = data.data
    #rospy.loginfo("I heard %s ticks from r_wheel",data.data)

def l_wheel_callback(data):
    global l_ticks
    l_ticks = data.data
    #rospy.loginfo("I heard %s ticks from l_wheel",data.data)

def imu_callback(data):
    global imu_ticks
    imu_ticks = float((data.data*71)/4068.0000)

rospy.init_node('odom_node')

rospy.Subscriber("r_wheel_node", Int32, r_wheel_callback)
rospy.Subscriber("l_wheel_node", Int32, l_wheel_callback)
rospy.Subscriber("imu_z_node", Float32, imu_callback)

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()


r_dist = 0
l_dist = 0
base_angl = 0
base_dist = 0

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
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, imu_ticks)
    
    r_dist = float(r_ticks/tpm)
    l_dist = float(l_ticks/tpm)

    base_angl =  (r_dist -l_dist) / base_wight
    base_dist = (r_dist + l_dist) / 2

    x = base_dist * math.cos(base_angl)
    y = base_dist * math.sin(base_angl)

    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    last_time = current_time
    r.sleep()



