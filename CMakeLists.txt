cmake_minimum_required(VERSION 2.8.3)
project(um482_ros_wrapper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  nmea_msgs
  nav_msgs
  sensor_msgs 
)

find_package(Eigen3 REQUIRED)

catkin_package(
	CATKIN_DEPENDS
		roscpp
		serial
		std_msgs
		nmea_msgs
		nav_msgs
		sensor_msgs 

)
#  INCLUDE_DIRS include
#  LIBRARIES serialPort
#  CATKIN_DEPENDS roscpp serial std_msgs
#  DEPENDS system_lib




include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_LIBS}
)


add_executable(um482_ros_wrapper_node src/main.cpp) 

add_dependencies(um482_ros_wrapper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

target_link_libraries(um482_ros_wrapper_node ${catkin_LIBRARIES} )

