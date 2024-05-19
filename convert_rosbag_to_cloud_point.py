import rospy
import rosbag
import sensor_msgs.point_cloud2 as pc2
import pcl
import pyntcloud
import pandas as pd
import os
import numpy as np
from scipy.spatial import Delaunay
import time

FILENAME = 'output'
ROOT_DIR = '/ros_ws/scripts'
OUTPUT_DIR = '/ros_ws/output'
BAGFILE = OUTPUT_DIR + '/' + FILENAME + '.bag'


def wait_for_ros_master():
    while not rospy.core.is_initialized():
        print("Waiting for ROS master to start...")
        time.sleep(1)

def save_pointcloud(msg):
    if msg._type == "sensor_msgs/PointCloud2":
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pc_list = list(pc_data)
        
        # Convert point cloud to numpy array
        points = np.array(pc_list)

        # Triangulate the point cloud
        triangles = Delaunay(points[:, :2])

        # Format the timestamp for the filename
        timestamp = msg.header.stamp.to_nsec()  # Converts the ROS timestamp to nanoseconds
        timestamp_str = str(timestamp)  # Converts the timestamp to string for file naming

        folder_path_obj = os.path.join(OUTPUT_DIR, '_obj')
        os.makedirs(folder_path_obj, exist_ok=True)
        obj_path = os.path.join(folder_path_obj, f"pointcloud_{timestamp_str}.obj")

        # Save triangles as OBJ file
        with open(obj_path, "w") as obj_file:
            for point in points:
                obj_file.write(f"v {point[0]} {point[1]} {point[2]}\n")
            for simplex in triangles.simplices:
                obj_file.write(f"f {simplex[0]+1} {simplex[1]+1} {simplex[2]+1}\n")

        # Create a PyntCloud object
        cloud = pyntcloud.PyntCloud(pd.DataFrame(points, columns=["x", "y", "z"]))

        folder_path_ply = os.path.join(OUTPUT_DIR, '_ply')
        os.makedirs(folder_path_ply, exist_ok=True)
        ply_path = os.path.join(folder_path_ply, f"pointcloud_{timestamp_str}.ply")

        # Alternatively, save PointCloud as PLY file
        cloud.to_file(ply_path, as_text=True)

def convert_rosbag_to_pointcloud(rosbag_path):
    wait_for_ros_master()
    bag = rosbag.Bag(rosbag_path, "r")

    pointcloud_count = 0  # Counter for each point cloud processed
    for topic, msg, _ in bag.read_messages(topics=["/sync/velodyne_points"]):
        save_pointcloud(msg)
        pointcloud_count += 1
        print(f"Processed point cloud {pointcloud_count} at time {msg.header.stamp.to_sec()}")
    bag.close()

if __name__ == "__main__":
    rospy.init_node("lidar_converter")
    rosbag_path = BAGFILE
    convert_rosbag_to_pointcloud(rosbag_path)
    print('PROCESS COMPLETED')

