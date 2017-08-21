cmake_minimum_required(VERSION 2.8.3)
project(kingfisher_msgs)

find_package(catkin REQUIRED message_generation)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Sense.msg
  Drive.msg
  Course.msg
  Helm.msg
)

generate_messages()

catkin_package( CATKIN_DEPENDS message_runtime )

