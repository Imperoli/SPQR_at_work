cmake_minimum_required (VERSION 2.8.7)
project (at-work-cfh-examples)

option(WithROS "WithROS" FALSE)

if(DEFINED _CATKIN_ALL_INCLUDED_)
  option(WithROS "WithROS" TRUE)
endif()

add_subdirectory(at_work_central_factory_hub_comm)

if(WithROS)
  message("Building ROS version of example robot")
  if(DEFINED _CATKIN_ALL_INCLUDED_)
    add_subdirectory(at_work_robot_example_ros)
  else()
    message("ROS example must be build using catkin_make")
  endif()
else()
  message("Building Non-ROS version of example robot")
  add_subdirectory(at_work_robot_example)
endif()
