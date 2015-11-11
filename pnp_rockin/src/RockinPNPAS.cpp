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

    register_action("wait",&RockinPNPActionServer::wait,this);
    register_action("move",&RockinPNPActionServer::move,this);

    listener = new tf::TransformListener();

    ac_move=NULL;

    PNP_cond_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 100);

}





