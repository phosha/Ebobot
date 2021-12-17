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
base_width = 0.19


r_ticks = 0
l_ticks = 0
imu_ticks = 0


r_dist = 0
l_dist = 0
base_angl = 0
base_dist = 0

Rddist = 0
Lddist = 0
Rold_dist = 0
Lold_dist = 0

vel_r = 0
vel_l = 0

def r_wheel_callback(data):
    global r_ticks, r_dist, Rddist, Rold_dist, vel_r
    r_ticks = data.data
    r_dist = r_ticks/tpm
    Rddist = r_dist - Rold_dist
    Rold_dist = r_dist
    vel_r = Rddist / 0.05

def l_wheel_callback(data):
    global l_ticks, l_dist, Lddist, Lold_dist, vel_l
    l_ticks = data.data
    l_dist = l_ticks/tpm
    Lddist = l_dist - Lold_dist
    Lold_dist = l_dist
    vel_l = Rddist / 0.05

def imu_callback(data):
    global imu_ticks
    imu_ticks = float((data.data*71)/4068.0000)

rospy.init_node('odom_node')

rospy.Subscriber("r_wheel_node", Int32, r_wheel_callback)
rospy.Subscriber("l_wheel_node", Int32, l_wheel_callback)
rospy.Subscriber("imu_z_node", Float32, imu_callback)

r_speed_pub = rospy.Publisher("r_speed", Float32,queue_size=50)
l_speed_pub = rospy.Publisher("l_speed", Float32,queue_size=50)


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

r = rospy.Rate(50.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    dt = current_time - last_time
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, imu_ticks)
    
    
    

    base_angl =  (r_dist -l_dist) / base_width
    base_dist = (r_dist + l_dist) / 2

    x = base_dist * math.cos(base_angl)
    y = base_dist * math.sin(base_angl)


    r_speed_pub.publish(vel_r)

   

    rospy.loginfo(vel_r)
    #rospy.loginfo(vel_l)

    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    last_time = current_time
    r.sleep()



