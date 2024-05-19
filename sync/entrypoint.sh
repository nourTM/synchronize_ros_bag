#!/bin/bash

# Source the ROS setup
source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash

# Start roscore in the background
roscore &
sleep 5  # Wait for roscore to start

# Execute the command passed to the Docker container
exec "$@"
