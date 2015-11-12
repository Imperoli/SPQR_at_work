#include <boost/thread/mutex.hpp>
#include <tf/transform_listener.h>

#include "RockinPNPAS.h"
#include "topics.h"

#include <math.h>
#include <Eigen/Dense>

#define RAD(a) ((a)/180.0*M_PI)
#define DEG(a) ((a)*180.0/M_PI)


using namespace std;

bool RockinPNPActionServer::getRobotPose(std::string robotname, double &x, double &y, double &th_rad) {
    if (listener==NULL) {
        listener = new tf::TransformListener();
    }

    string src_frame = "/map";
    string dest_frame = "/" + robotname + "/base_frame";
    if (robotname=="") { // local trasnformation
        src_frame = "map";
        dest_frame = "base_link";
    }

    tf::StampedTransform transform;
    try {
        listener->waitForTransform(src_frame, dest_frame, ros::Time(0), ros::Duration(3));
        listener->lookupTransform(src_frame, dest_frame, ros::Time(0), transform);
    }
    catch(tf::TransformException ex) {
        th_rad = 999999;
        ROS_ERROR("Error in tf trasnform %s -> %s\n",src_frame.c_str(), dest_frame.c_str());
        ROS_ERROR("%s", ex.what());
        return false;
    }
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    th_rad = tf::getYaw(transform.getRotation());

    return true;
}


bool RockinPNPActionServer::getLocationPosition(string loc, float &GX, float &GY, float &Gtheta) {


    if (loc=="Workstation1") {
        GX = -3.05; GY = -0.95; Gtheta=3.14159;
        ROS_INFO_STREAM("Location " << loc << " at " << GX  << " , " << GY);
    }
    else if (loc=="Workstation2") {
        GX = -4.35; GY = -3.05; Gtheta=0.0;
        ROS_INFO_STREAM("Location " << loc << " at " << GX  << " , " << GY);      
    }
    else if (loc=="Workstation3") {
        GX = -2.7; GY = -2.25; Gtheta=(3.14159/2);
        ROS_INFO_STREAM("Location " << loc << " at " << GX  << " , " << GY);      
    }
    else {
        ROS_ERROR_STREAM("Location "<<loc<<" unknown.");
        return false;
    }

    return true;
}



/*
 * ACTIONS
 */


void RockinPNPActionServer::move(string params, bool *run) {
  ROS_INFO("move started");
  float GX,GY,Gtheta;
  if (getLocationPosition(params,GX,GY,Gtheta)) {
    do_move(GX,GY,Gtheta,run);
  }
  else 
    ROS_WARN("Advertise: Cannot find location %s.",params.c_str());
}


void RockinPNPActionServer::wait(string params, bool *run)
{
    // ROS_INFO_STREAM("### Executing Wait action " << params << " ... ");
/*
    int sleeptime=1; // * 0.2 sec.
    while (*run && sleeptime-->0)
        ros::Duration(0.2).sleep();
*/
    ROS_INFO("Just waiting ...");
    ros::Duration(50).sleep();
/*
    if (*run)
        ROS_INFO("### Finished Wait");
    else
        ROS_INFO("### Aborted Wait");
*/
}


void RockinPNPActionServer::do_move(float GX, float GY, float GTh_RAD, bool *run) { // theta in degrees
  ROS_INFO("do_move started");
  mtx_movebase.lock();

  if (ac_move==NULL) { //create the client only once
    // Define the action client (true: we want to spin a thread)
    ac_move = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(TOPIC_MOVE_BASE, true);

    // Wait for the action server to come up
    while(!ac_move->waitForServer(ros::Duration(5.0))){
	    ROS_INFO("Waiting for move_base action server to come up");
    }
  }

  // Read time
  double secs =ros::Time::now().toSec();
  while (secs==0) {  // NEEDED OTHERWISE CLOCK WILL BE 0 AND GOAL_ID IS NOT SET CORRECTLY
	  ROS_ERROR_STREAM("Time is null: " << ros::Time::now());
	  ros::Duration(1.0).sleep();
      secs =ros::Time::now().toSec();
  }

  std::cout<<"x "<<GX<<" y "<<GY<<" GO"<<std::endl;
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "map";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x=GX;
  target_pose.pose.position.y=GY;
  
  Eigen::Quaternionf q(Eigen::AngleAxisf(GTh_RAD, Eigen::Vector3f::UnitZ()));
  target_pose.pose.orientation.x = q.x();
  target_pose.pose.orientation.y = q.y();
  target_pose.pose.orientation.z = q.z();
  target_pose.pose.orientation.w = q.w();
  //std::cout<<"o.x "<<target_pose.pose.orientation.x<<" o.y "<<target_pose.pose.orientation.y<<" o.z "<<target_pose.pose.orientation.z<<" o.w "<<target_pose.pose.orientation.w<<" GO"<<std::endl;
  
  move_base_msgs::MoveBaseGoal move_base_goal;
  move_base_goal.target_pose = target_pose;
  
  ac_move->sendGoal(move_base_goal);
  ac_move->waitForResult();

  if(ac_move->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("cannot move the base for some reason (step 1)");
    string param = "PNPconditionsBuffer/goalReached";
    handle.setParam(param, 0);
  }else{
    string param = "PNPconditionsBuffer/goalReached";
    handle.setParam(param, 1);
  }

#if 0
  // Print result
  if (!(*run))
    ROS_INFO("External interrupt!!!");
  else if (d<=d_threshold) 
    ROS_INFO("Target reached (Internal check)");
  else if (ac_movebase->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base failed to reach the move_base goal for some reason");
  else
    ROS_INFO("!move_base goal reached!");
#endif

  // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
  ac_move->cancelAllGoals(); ros::Duration(0.2).sleep(); // wait a little

  mtx_movebase.unlock();
}
