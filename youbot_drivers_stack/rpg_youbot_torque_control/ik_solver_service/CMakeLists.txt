cmake_minimum_required(VERSION 2.8.3)
project(ik_solver_service)

find_package(catkin REQUIRED
  COMPONENTS
	roscpp
	message_generation
	geometry_msgs
	std_msgs
	rpg_youbot_common
	youbot_arm_model
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
	FILES
	FullyConstrainedReq.msg
	FullyConstrainedRes.msg
)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	SolveClosestIK.srv
	SolveFullyConstrainedIK.srv
	SolveFullyConstrainedIKArray.srv
	SolvePreferredPitchIK.srv
	SolvePreferredTypeIK.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
	geometry_msgs
	std_msgs
	message_runtime
	rpg_youbot_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
  /usr/include/eigen3/
)
#message("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO ${Eigen_INCLUDE_DIRS} OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")

add_executable(ik_solver_service src/ik_solver_service.cpp src/youbot_grasp_ik.cpp)
add_dependencies(ik_solver_service ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(ik_solver_service
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(test_client src/test_client.cpp)
add_dependencies(test_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_client
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)
