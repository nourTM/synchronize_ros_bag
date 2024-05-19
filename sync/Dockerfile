# Use an official ROS base image
FROM ros:melodic

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    ros-melodic-rosbag \
    ros-melodic-message-filters \
    ros-melodic-sensor-msgs \
    build-essential \
    cmake

# Create and initialize the catkin workspace
WORKDIR /catkin_ws

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  mkdir -p /catkin_ws/src && \
                  cd /catkin_ws && \
                  catkin_make"

# Copy the bag file into the container
#COPY sensor_data_nour.bag /catkin_ws/

# Copy your ROS package into the container
COPY sync_data_pack /catkin_ws/src/sync_data_pack

# Build your ROS package
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  cd /catkin_ws && \
                  catkin_make && \
                  source devel/setup.bash"

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh

# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

# Set the entrypoint script to be executed
ENTRYPOINT ["/entrypoint.sh"]

# Run the ROS node with input and output bag file arguments
CMD ["/bin/bash", "-c", "rosrun sync_data_pack sync_data_pack_node /output/sensor_data_nour.bag /output/output.bag"]
