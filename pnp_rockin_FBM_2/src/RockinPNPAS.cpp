#include <boost/thread/thread.hpp>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/algorithm/string.hpp>


#include "topics.h"

#include "RockinPNPAS.h"

RockinPNPActionServer::RockinPNPActionServer(ros::NodeHandle n) : PNPActionServer(), handle(n), handlep("~")  
{
    event_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 10);
    plantoexec_pub = handle.advertise<std_msgs::String>(TOPIC_PLANTOEXEC, 100);
    //rcom_pub= handle.advertise<tcp_interface::RCOMMessage>(TOPIC_RCOMMESSAGE,10);
    feedback_pub = handle.advertise<at_work_robot_example_ros::BenchmarkFeedback>(TOPIC_CFH_BENCHMARK_FEEDBACK, 1);

    //cfh subscribers initialization
    sub_cfh_inventory = handle.subscribe(TOPIC_CFH_INVENTORY, 1, &RockinPNPActionServer::cb_cfh_inventory, this);
    sub_cfh_order = handle.subscribe(TOPIC_CFH_ORDER, 1, &RockinPNPActionServer::cb_cfh_order, this);
    //sub_cfh_conveyor_belt_command = handle.subscribe(TOPIC_CFH_CONVEYOR_BELT_COMMAND, 1, &RockinPNPActionServer::cb_cfh_conveyor_belt_command);
    sub_cfh_conveyor_belt_status = handle.subscribe(TOPIC_CFH_CONVEYOR_BELT_STATUS, 1, &RockinPNPActionServer::cb_cfh_conveyor_belt_status, this);
    //sub_cfh_drill_machine_status = handle.subscribe(TOPIC_CFH_DRILL_MACHINE_STATUS, 1, &RockinPNPActionServer::cb_drill_machine_status);
    //sub_cfh_drilling_machine_command = handle.subscribe(TOPIC_CFH_DRILLING_MACHINE_COMMAND, 1, &RockinPNPActionServer::cb_drilling_machine_command);
    sub_cfh_benchmark_state = handle.subscribe(TOPIC_CFH_BENCHMARK_STATE, 1, &RockinPNPActionServer::cb_cfh_benchmark_state, this);

    //pnp registering actions
    register_action("init",&RockinPNPActionServer::init,this);
    register_action("wait",&RockinPNPActionServer::wait,this);
    register_action("move",&RockinPNPActionServer::move,this);
    register_action("detection",&RockinPNPActionServer::detection,this);
    register_action("grasp",&RockinPNPActionServer::grasp,this);
    register_action("drop",&RockinPNPActionServer::drop,this);
    register_action("verifyGrasp",&RockinPNPActionServer::verifyGrasp,this);
    register_action("detectPlane",&RockinPNPActionServer::detectPlane,this);
    register_action("dropOnPlane",&RockinPNPActionServer::dropOnPlane,this);

    listener = new tf::TransformListener();

    ac_move=NULL;
    ac_detection=NULL;
    ac_manipulation=NULL;

    //counter for detection
    counter_detection=0;
    counter_verify_grasp=0;

    //bring biring_box
    bring_biring_box = true;

    call_flag=true;

    //tools_idx
    tools_idx = 0;

    start_plan=true;
    stop_plan = true;

    //output debug
    out_debug = true;
    PNP_cond_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 100);

    computeTransformation("arm_base", "kinect_rgb_optical_frame", T_kinect2arm);
    if(out_debug)
  	 std::cout<<"T_kinect2arm:\n"<<T_kinect2arm.matrix()<<std::endl;

}





