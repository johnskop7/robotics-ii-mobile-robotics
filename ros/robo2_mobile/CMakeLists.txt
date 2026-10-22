cmake_minimum_required(VERSION 3.0.2)
project(robo2_mobile)

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib rospy std_msgs geometry_msgs sensor_msgs
)
catkin_package(CATKIN_DEPENDS roscpp roslib rospy std_msgs geometry_msgs sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(random_walker src/random_walker.cpp)
target_link_libraries(random_walker ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(random_walker ${catkin_EXPORTED_TARGETS})
endif()

catkin_install_python(PROGRAMS
  scripts/follower.py
  scripts/localization.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS random_walker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
