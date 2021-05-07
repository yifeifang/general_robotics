cmake_minimum_required(VERSION 3.7.2)
project(robotics_labs)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg pcl_conversions pcl_ros)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

install(
  PROGRAMS
    scripts/demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch maps config msg srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(segmentation src/segmentation.cpp)
target_link_libraries(segmentation ${catkin_LIBRARIES})
add_dependencies(segmentation ${catkin_EXPORTED_TARGETS})
