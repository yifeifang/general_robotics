cmake_minimum_required(VERSION 3.7.2)
project(robotics_labs)

find_package(catkin)
catkin_package()

install(
  PROGRAMS
    scripts/demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch maps config msg srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
