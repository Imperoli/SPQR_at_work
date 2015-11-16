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

    register_action("init",&RockinPNPActionServer::init,this);
    register_action("wait",&RockinPNPActionServer::wait,this);
    register_action("move",&RockinPNPActionServer::move,this);
    register_action("detection",&RockinPNPActionServer::detection,this);
    register_action("grasp",&RockinPNPActionServer::grasp,this);

    listener = new tf::TransformListener();

    ac_move=NULL;
    ac_detection=NULL;
    ac_grasping=NULL;

    PNP_cond_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 100);

    computeTransformation("arm_base", "kinect_rgb_optical_frame", T_kinect2arm);
  std::cout<<"T_kinect2arm:\n"<<T_kinect2arm.matrix()<<std::endl;

}





