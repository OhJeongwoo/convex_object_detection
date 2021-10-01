cmake_minimum_required(VERSION 2.8.3)
project(convex_object_detection)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  rosbag
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  cv_bridge
  message_filters
  tf
  tf2
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3)
find_package(PCL 1.8 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES 
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs
	DEPENDS Eigen
	opencv_highgui opencv_imgproc
  INCLUDE_DIRS include
	#LIBRARIES ${PROJECT_NAME}
)

find_package(OpenCV REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
 )

#add_library({LIB_NAME} lib/{LIB_NAME}.cpp)

set (SOURCES
 #${CMAKE_CURRENT_SOURCE_DIR}/lib/{LIB_NAME}.cpp
 #${CMAKE_CURRENT_SOURCE_DIR}/lib/utils.cpp
)

set (HEADERS
 #${CMAKE_CURRENT_SOURCE_DIR}/include/{LIB_NAME}.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/utils.h
)


add_executable(object_detector src/object_detector.cpp ${HEADERS} ${SOURCES})
target_link_libraries(object_detector ${catkin_LIBRARIES} ${CV_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
add_dependencies(object_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/npy2pcd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


