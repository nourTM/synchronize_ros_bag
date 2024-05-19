# Use the official ROS base image
FROM ros:noetic-ros-base

# Install additional dependencies
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip3 install opencv-python-headless\
    pip install pyntcloud\
    pip install Cython\
    pip install --upgrade numpy

RUN pip install pcl\
    pip install scipy

# Install cv_bridge and other necessary ROS packages
RUN apt-get install -y ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-velodyne-pointcloud

# Create a directory for your scripts
RUN mkdir -p /ros_ws/scripts /ros_ws/output

# Copy your Python script to the Docker image
COPY convert_rosbag_to_images.py /ros_ws/scripts/
COPY convert_rosbag_to_cloud_point.py /ros_ws/scripts/
COPY convert_rosbag_to_imu_gnss.py /ros_ws/scripts/


# Set the working directory
WORKDIR /ros_ws

# Source ROS setup.bash
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Run the Python script when the container starts
#CMD ["roscore", "&", "python3", "scripts/convert_rosbag_to_cloud_point.py"]

#CMD ["bash", "-c", "roscore & sleep 5 && python3 scripts/convert_rosbag_to_images.py"]
CMD ["bash", "-c", "roscore & sleep 5 && python3 scripts/convert_rosbag_to_cloud_point.py"]

#CMD ["bash", "-c", "roscore & sleep 5 && python3 scripts/convert_rosbag_to_imu_gnss.py"]
