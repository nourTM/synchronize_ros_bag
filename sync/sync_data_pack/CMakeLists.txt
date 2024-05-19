cmake_minimum_required(VERSION 3.0.2)
project(sync_data_pack)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
  rosbag 
)

catkin_package(
  # Include this line if you're using messages or services
  # CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/sync_all_data.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}  # This links against all catkin libraries
)
