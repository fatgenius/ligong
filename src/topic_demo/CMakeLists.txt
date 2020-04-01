cmake_minimum_required(VERSION 2.8.3)

project(topic_demo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  # cv_bridge
  message_generation
  roscpp
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   gps.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES publish_subscribe_demo
  CATKIN_DEPENDS  roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ./inc
  ./tinyxml_lib/inc
)


add_executable(talker src/node/talker.cpp )
add_dependencies(talker topic_demo_generate_messages_cpp)
target_link_libraries(talker ${catkin_LIBRARIES})


add_executable(listener src/node/listener.cpp )
add_dependencies(listener topic_demo_generate_messages_cpp)
target_link_libraries(listener ${catkin_LIBRARIES})


find_package(OpenCV 3 REQUIRED)
add_executable(rp_goal 
  ./src/node/rp_goal_node.cpp
  ./src/rp_goal.cpp
  ./tinyxml_lib/src/tinystr.cpp
  ./tinyxml_lib/src/tinyxml.cpp
  # ./tinyxml_lib/src/tinyxmlerrors.cpp
  ./tinyxml_lib/src/tinyxmlparser.cpp
)
target_link_libraries(rp_goal 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)


add_executable(comb2speed src/node/comb2speed_node.cpp
  ./src/comb2speed.cpp)
target_link_libraries(comb2speed 
  ${catkin_LIBRARIES} 
)


add_executable(tran_vo_odom ./src/node/tran_vo_odom_node.cpp)
target_link_libraries(tran_vo_odom
  ${catkin_LIBRARIES} 
)


add_executable(meas_fusion ./src/node/meas_fusion_node.cpp)
target_link_libraries(meas_fusion
  ${catkin_LIBRARIES} 
)


add_executable(cv_map ./src/node/cv_map_node.cpp
  ./src/ros_map_use.cpp)
target_link_libraries(cv_map
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)