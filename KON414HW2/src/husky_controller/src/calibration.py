#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import message_filters


# Convert ROS PointCloud2 to Open3D PointCloud
def ros_to_open3d(ros_cloud):
    points = list(pc2.read_points(ros_cloud, skip_nans=True))
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(np.array(points)[:, :3])
    return o3d_cloud

# Transform PointCloud2 to the desired frame
def transform_pointcloud(msg, target_frame, tf_buffer):
    try:
        # Transform the point cloud
        transformed_msg = tf_buffer.transform(msg, target_frame, rospy.Duration(1.0))
        return transformed_msg
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"TF transform failed: {e}")
        return None

def callback(lidar_msg, rgbd_msg, tf_buffer):
    print("Callback triggered!")  # Debug line

    # Transform point clouds to the 'base_link' frame
    lidar_msg = transform_pointcloud(lidar_msg, "base_link", tf_buffer)
    rgbd_msg = transform_pointcloud(rgbd_msg, "base_link", tf_buffer)

    if lidar_msg is None or rgbd_msg is None:
        print("Transformation failed, skipping this frame.")
        return

    # Convert ROS messages to Open3D point clouds
    lidar_cloud = ros_to_open3d(lidar_msg)
    rgbd_cloud = ros_to_open3d(rgbd_msg)

    # Visualize initial alignment
    o3d.visualization.draw_geometries([lidar_cloud.paint_uniform_color([1, 0, 0]),
                                       rgbd_cloud.paint_uniform_color([0, 1, 0])])

    # Perform ICP alignment
    threshold = 0.2  # Maximum correspondence distance in meters
    result = o3d.pipelines.registration.registration_icp(
        rgbd_cloud, lidar_cloud, threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    # Check if ICP succeeded
    if result.fitness > 0.2:  # Fitness threshold
        print("ICP succeeded!")
        print("Transformation matrix:")
        print(result.transformation)

        # Apply the transformation to the RGB-D cloud and visualize
        rgbd_cloud.transform(result.transformation)
        o3d.visualization.draw_geometries([lidar_cloud.paint_uniform_color([1, 0, 0]),
                                           rgbd_cloud.paint_uniform_color([0, 1, 0])])
    else:
        print("ICP failed. Ensure the point clouds have sufficient overlap.")

if __name__ == "__main__":
    rospy.init_node("lidar_camera_calibration", anonymous=True)

    # TF buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Subscribe to LiDAR and RGB-D point cloud topics
    lidar_sub = message_filters.Subscriber("/robosense_points2", PointCloud2)
    rgbd_sub = message_filters.Subscriber("/depth/points", PointCloud2)

    # Synchronize the topics
    sync = message_filters.ApproximateTimeSynchronizer([lidar_sub, rgbd_sub], queue_size=10, slop=0.1)
    sync.registerCallback(callback, tf_buffer)

    print("Calibration node is running...")  # Debug line
    rospy.spin()

