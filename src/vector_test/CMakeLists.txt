cmake_minimum_required(VERSION 3.0.2)
project(vector_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vector_test
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf geometry_msgs
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME} src/main.cpp)

 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )