CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(rviz_amrl_msgs_visualization)


INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()

FIND_PACKAGE(Qt5 COMPONENTS Core Gui REQUIRED)
# FIND_PACKAGE(catkin COMPONENTS message_runtime message_generation std_msgs roscpp rviz geometry_msgs amrl_msgs)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Werror")

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# catkin_package(
#   LIBRARIES rviz_amrl_msgs_visualization
#   CATKIN_DEPENDS message_runtime std_msgs rviz geometry_msgs amrl_msgs)

# INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

ROSBUILD_ADD_LIBRARY(rviz_amrl_msgs_visualization 
    src/amrl_msgs_visualization.cc)
TARGET_LINK_LIBRARIES(rviz_amrl_msgs_visualization Qt5::Core Qt5::Gui)
# ADD_DEPENDENCIES(rviz_amrl_msgs_visualization ${catkin_EXPORTED_TARGETS})


