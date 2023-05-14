cmake_minimum_required(VERSION 2.8.3)
project(cyton_gamma_300_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  control_toolbox
  )

find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(DEPENDS
    roscpp
    gazebo_ros
    control_toolbox
  )

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include)

add_library(gazebo_mimic_joint_plugin src/gazebo_mimic_joint_plugin.cpp)
target_link_libraries(gazebo_mimic_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
