CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

PROJECT(annotation_tool)



find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)
find_package(PCL 1.4 REQUIRED COMPONENTS visualization filters segmentation)
find_package(Qt4 4.7 REQUIRED)
FIND_PACKAGE(VTK REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${QT_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})

include(${QT_USE_FILE})

#set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ./../../bin/)

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES compression_node
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

ADD_SUBDIRECTORY(src)
