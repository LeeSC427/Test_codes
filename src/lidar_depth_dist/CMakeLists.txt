cmake_minimum_required(VERSION 3.0.2)
project(lidar_depth_dist)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lidar_depth_dist
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME} src/main.cpp)

 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )