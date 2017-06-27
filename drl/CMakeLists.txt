cmake_minimum_required(VERSION 2.8.3)
project(deep_reinforced_landing)

find_package(catkin REQUIRED COMPONENTS
  message_generation
	roscpp
	rospy
	geometry_msgs
	sensor_msgs
	std_msgs
  gazebo_msgs
  ardrone_autonomy
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED
)

add_message_files(DIRECTORY msg
  FILES
  reward_req.msg
  camera_req.msg
  reset_req.msg
)

add_service_files(DIRECTORY srv
  FILES
  GetDoneAndReward.srv
  GetCameraImage.srv
  ResetPosition.srv
  NewCameraService.srv
  SendCommand.srv
  GetRelativePose.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  gazebo_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES boundingBox utilities
  CATKIN_DEPENDS message_runtime roscpp rospy geometry_msgs sensor_msgs std_msgs gazebo_msgs ardrone_autonomy
)



include_directories(include
   ${catkin_INCLUDE_DIRS}
)
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination

add_library(boundingBox src/boundingBox.cpp )
add_library(utilities src/utilities.cpp)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(drl_services src/drl_services_node.cpp)
target_link_libraries(drl_services
   ${catkin_LIBRARIES}
   boundingBox
   utilities
)

## Build service client and server
# %Tag(SRVCLIENT)%
#add_executable(reward_server src/reward_node.cpp)
#target_link_libraries(reward_server ${catkin_LIBRARIES})
add_dependencies(drl_services ${catkin_EXPORTED_TARGETS})
#add_dependencies(drl deep_reinforced_landing_gencpp)
# %EndTag(SRVCLIENT)%



list( APPEND CMAKE_CXX_FLAGS " -std=c++11 ${CMAKE_CXX_FLAGS} -O3 -g  -ftest-coverage -fprofile-arcs ")

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
