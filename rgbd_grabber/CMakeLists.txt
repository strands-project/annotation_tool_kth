cmake_minimum_required(VERSION 2.8.3)
project(rgbd_grabber)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)
find_package( OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)

set(CMAKE_CXX_FLAGS "-fpermissive -O3 -g -fPIC -std=c++0x")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES compression_node
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rgbd_grabber src/rgbd_grabber.cpp src/main.cpp include/rgbd_grabber.h)


target_link_libraries(rgbd_grabber
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )

