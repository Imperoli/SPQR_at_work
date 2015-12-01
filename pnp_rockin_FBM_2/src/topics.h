// defines the names of the topics to send data between modules

#ifndef __TOPICS__
#define __TOPICS__

#define TOPIC_ROBOT_LOCATION "amcl_pose"
#define TOPIC_LASER_OBSMAP "laser_obstacle_map"

#define TOPIC_MOVE_BASE "goto"
#define TOPIC_DETECTION_NODE "/detect_objects"
#define TOPIC_GRASPING_NODE "/arm_planner"
#define TOPIC_TURN "turn"
#define TOPIC_FOLLOWCORRIDOR "follow_corridor"
#define TOPIC_SAY "say"
#define TOPIC_STAGE_SAY "stage_say"

#define TOPIC_PLANTOEXEC "planToExec"
#define TOPIC_PNPACTIVEPLACES "pnp/currentActivePlaces"
#define TOPIC_PNPCONDITION "PNPConditionEvent"
#define PARAM_PNPCONDITIONBUFFER "PNPconditionsBuffer/"

#define TOPIC_RCOMMESSAGE "/RCOMMessage"

//cfh topics
#define TOPIC_CFH_INVENTORY "/robot_example_ros/inventory"
#define TOPIC_CFH_ORDER "/robot_example_ros/order_info"
#define TOPIC_CFH_CONVEYOR_BELT_COMMAND "/robot_example_ros/conveyor_belt_command"
#define TOPIC_CFH_CONVEYOR_BELT_STATUS "/robot_example_ros/conveyor_belt_status"
#define TOPIC_CFH_DRILL_MACHINE_STATUS "/robot_example_ros/drill_machine_status"
#define TOPIC_CFH_DRILLING_MACHINE_COMMAND "/robot_example_ros/drilling_machine_command"
#define TOPIC_CFH_BENCHMARK_STATE "/robot_example_ros/benchmark_state"
#define TOPIC_CFH_BENCHMARK_FEEDBACK "/robot_example_ros/benchmark_feedback"


#endif
