cmake_minimum_required(VERSION 2.8.3)
project(topic_logger)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  message_filters
)

catkin_package(
		DEPENDS Eigen
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC} src/main.cpp)
set(SRC ${SRC} src/TopicLogger.cpp)
set(SRC ${SRC} include/TopicLogger.h)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

add_executable (topic_logger_node ${SRC} )
target_link_libraries (topic_logger_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries (topic_logger_node ${Eigen_LIBRARIES} )
