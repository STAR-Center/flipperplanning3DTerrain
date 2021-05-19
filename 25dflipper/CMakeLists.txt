cmake_minimum_required(VERSION 2.8.3)
project(flipper25d)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Float64StampedMultiArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)




#add_executable(JoyControl src/joyControll.cpp)
#add_executable(JoyControlSony src/joyControll_sony.cpp)
#target_link_libraries(JoyControl ${catkin_LIBRARIES})
#target_link_libraries(JoyControlSony ${catkin_LIBRARIES})
