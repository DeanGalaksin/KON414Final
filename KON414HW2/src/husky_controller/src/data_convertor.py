#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
from sensor_msgs import point_cloud2

def pointcloud_callback(msg):
    points = [(p.x, p.y, p.z) for p in msg.points]
    header = msg.header
    cloud2 = point_cloud2.create_cloud_xyz32(header, points)
    pub.publish(cloud2)

rospy.init_node('pointcloud_to_pointcloud2')
rospy.Subscriber('/robosense_points', PointCloud, pointcloud_callback)
pub = rospy.Publisher('/robosense_points2', PointCloud2, queue_size=10)
rospy.spin()

