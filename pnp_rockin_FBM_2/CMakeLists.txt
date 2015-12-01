cmake_minimum_required(VERSION 2.8.3)
project(pnp_rockin_fbm2)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pnp_ros
  tf
  tf_conversions
  eigen_conversions
  at_work_robot_example_ros
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${pnp_ros_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/include/eigen3/
)

add_executable(pnp_rockin_node_fbm2 src/main.cpp src/RockinPNPAS.cpp src/Actions.cpp src/Conditions.cpp)

target_link_libraries(pnp_rockin_node_fbm2 ${pnp_ros_LIBRARIES} ${catkin_LIBRARIES})
