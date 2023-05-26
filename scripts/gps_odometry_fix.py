#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry

pub_gps = rospy.Publisher('odometry/gps_fix', Odometry, queue_size=600);

def callback(gps_odom):

    gps_odom.pose.pose.position.x=gps_odom.pose.pose.position.x
    gps_odom.pose.pose.position.y=-gps_odom.pose.pose.position.y
    gps_odom.pose.pose.position.z=-gps_odom.pose.pose.position.z    

    gps_odom.pose.pose.orientation.x= gps_odom.pose.pose.orientation.x
    gps_odom.pose.pose.orientation.y=-gps_odom.pose.pose.orientation.y
    gps_odom.pose.pose.orientation.z=-gps_odom.pose.pose.orientation.z    
    gps_odom.pose.pose.orientation.w=gps_odom.pose.pose.orientation.w    

    pub_gps.publish(gps_odom)


def gps_listener():
    rospy.init_node("gps_listener", anonymous=True)
    nodeName = rospy.get_name();
    print("\033[1;36m\n>>> GPS odometry fix <<<\033[0m")

    rospy.Subscriber('odometry/gps',Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    gps_listener()

