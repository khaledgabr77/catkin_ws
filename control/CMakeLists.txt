cmake_minimum_required(VERSION 3.0.2)
project(control)
## Find catkin and any catkin packages
#roscpp rospy geometry_msgs genmsg
#geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs

  find_package(catkin REQUIRED COMPONENTS  geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs rospy genmsg)
 add_definitions(-lwiringPi)
   
   ## Declare ROS messages and services
   #add_message_files(FILES Num.msg)
   #add_service_files(FILES AddTwoInts.srv)
 
  ## Generate added messages and services
  generate_messages(DEPENDENCIES geometry_msgs)
 
  ## Declare a catkin package
#()was empty
  catkin_package()
 
  ## Build talker and listener
#${Boost_INCLUDE_DIRS}
  include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
 
  add_executable(talker src/talker.cpp)
  target_link_libraries(talker  ${catkin_LIBRARIES})
  add_dependencies(talker control_generate_messages_cpp)
 
  add_executable(listener src/listener.cpp)
  target_link_libraries(listener wiringPi -lrt -lcrypt ${catkin_LIBRARIES})
  add_dependencies(listener control_generate_messages_cpp)
