import rospy
import rosbag
import csv
import os
import time
from sensor_msgs.msg import Imu, NavSatFix

FILENAME = 'output'
ROOT_DIR = '/ros_ws/scripts'
OUTPUT_DIR = '/ros_ws/output'
BAGFILE = os.path.join(OUTPUT_DIR, FILENAME + '.bag')

def wait_for_ros_master():
    while not rospy.core.is_initialized():
        print("Waiting for ROS master to start...")
        time.sleep(1)

def save_imu_data(msg):
    timestamp = msg.header.stamp.to_nsec()  # Get timestamp in nanoseconds
    folder_path = os.path.join(OUTPUT_DIR, '_imu')
    os.makedirs(folder_path, exist_ok=True)
    csv_path = os.path.join(folder_path, f"imu_data_{timestamp}.csv")

    # Write IMU data to CSV
    with open(csv_path, mode="w", newline="") as csvfile:
        fieldnames = ["timestamp", "orientation_x", "orientation_y", "orientation_z", "orientation_w",
                      "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
                      "linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z"]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow({
            "timestamp": msg.header.stamp.to_sec(),
            "orientation_x": msg.orientation.x,
            "orientation_y": msg.orientation.y,
            "orientation_z": msg.orientation.z,
            "orientation_w": msg.orientation.w,
            "angular_velocity_x": msg.angular_velocity.x,
            "angular_velocity_y": msg.angular_velocity.y,
            "angular_velocity_z": msg.angular_velocity.z,
            "linear_acceleration_x": msg.linear_acceleration.x,
            "linear_acceleration_y": msg.linear_acceleration.y,
            "linear_acceleration_z": msg.linear_acceleration.z
        })

def save_gnss_data(msg):
    timestamp = msg.header.stamp.to_nsec()  # Get timestamp in nanoseconds
    folder_path = os.path.join(OUTPUT_DIR, '_gnss')
    os.makedirs(folder_path, exist_ok=True)
    csv_path = os.path.join(folder_path, f"gnss_data_{timestamp}.csv")

    # Write GPS data to CSV
    with open(csv_path, mode="w", newline="") as csvfile:
        fieldnames = ["timestamp", "latitude", "longitude", "altitude"]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow({
            "timestamp": msg.header.stamp.to_sec(),
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude
        })

def convert_rosbag_to_imu_and_gps(rosbag_path):
    wait_for_ros_master()
    bag = rosbag.Bag(rosbag_path, "r")

    for topic, msg, _ in bag.read_messages(topics=["/sync/imu", "/sync/gnss"]):
        if topic == "/sync/imu":
            save_imu_data(msg)
        elif topic == "/sync/gnss":
            save_gnss_data(msg)

    bag.close()

if __name__ == "__main__":
    rospy.init_node("gps_imu_extractor")
    rosbag_path = BAGFILE
    convert_rosbag_to_imu_and_gps(rosbag_path)
    print('PROCESS COMPLETED')

