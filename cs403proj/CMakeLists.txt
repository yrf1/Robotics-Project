PROJECT(cs403proj)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	pcl_conversions
	pcl_ros
)

find_package(OpenCV)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include_directories(
	${OpenCV_INCLUDE_DIRS}
)

ROSBUILD_INIT()

ROSBUILD_ADD_BOOST_DIRECTORIES()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(libs roslib roscpp)

ROSBUILD_ADD_EXECUTABLE(cs403proj src/cs403proj.cpp)
TARGET_LINK_LIBRARIES(cs403proj ${libs} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
ROSBUILD_LINK_BOOST(cs403proj system)
