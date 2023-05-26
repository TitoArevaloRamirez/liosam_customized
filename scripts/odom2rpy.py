#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import pandas as pd
from datetime import datetime
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pub = rospy.Publisher('/rpy', Odometry, queue_size=60);

def callback(odom):

    rawData = []
    
    Q = odom.pose.pose.orientation
    orientation_list = [Q.x, Q.y, Q.z, Q.w]

    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    odom.pose.pose.orientation.x = roll
    odom.pose.pose.orientation.y = pitch
    odom.pose.pose.orientation.z = yaw

    pub.publish(odom)


def odom_listener():
    rospy.init_node('Q2rpy', anonymous=True)
    nodeName = rospy.get_name();
    print("\033[1;36m\n>>> " + nodeName + " started <<<\033[0m")

    rospy.Subscriber('/odom_q',Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    odom_listener()
