cmake_minimum_required(VERSION 2.8.3)
project(subodm)
## Find catkin and any catkin packages
  find_package(catkin REQUIRED COMPONENTS roscpp )

   
 
  ## Declare a catkin package
  catkin_package()
 
  ## Build subodm
  include_directories(include ${catkin_INCLUDE_DIRS} )
 

 
  add_executable(subodm src/subodm.cpp)
  target_link_libraries(subodm ${catkin_LIBRARIES})
  add_dependencies(subodm control_generate_messages_cpp)
