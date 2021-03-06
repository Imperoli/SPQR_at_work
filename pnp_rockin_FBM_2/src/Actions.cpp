#include <boost/thread/mutex.hpp>
#include <tf/transform_listener.h>

#include "RockinPNPAS.h"
#include "topics.h"
#include <string>
#include <sstream>

#include <math.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#define RAD(a) ((a)/180.0*M_PI)
#define DEG(a) ((a)*180.0/M_PI)


using namespace std;

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

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

void RockinPNPActionServer::PoseArrayToEigen(const geometry_msgs::PoseArray& poses, std::vector<Eigen::Affine3d>& output)
{
  output.clear();
  for(int i=0; i<poses.poses.size(); i++){
    Eigen::Affine3d out;
    tf::poseMsgToEigen(poses.poses[i],out);
    output.push_back(out);
  }
}

void RockinPNPActionServer::tf2Affine(tf::StampedTransform& tf, Eigen::Affine3d& T)
{
  tf::Vector3 o=tf.getOrigin();
  tf::Quaternion q_tf=tf.getRotation();
  Eigen::Quaterniond q(q_tf[3],q_tf[0],q_tf[1],q_tf[2]);
  Eigen::Matrix3d R(q);
  Eigen::Vector3d t(o[0],o[1],o[2]);
  T.linear()=R; T.translation()=t;
  
}

bool RockinPNPActionServer::computeTransformation(std::string target, std::string source, Eigen::Affine3d& T)
{
  //tf::TransformListener listener;
  tf::StampedTransform transform;
  for (int i=0; i<3;i++)
  {
    try
    {
      ros::Time now=ros::Time::now();
      listener->waitForTransform( target, source, now, ros::Duration(.25));
      listener->lookupTransform( target, source, now, transform);
      //tf::transformTFToEigen(transform, T);
      tf2Affine(transform, T);
      return true;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
  }
  return false;
}

void RockinPNPActionServer::read_workstations_locations(std::map<std::string, Eigen::Vector3f>& ws)
{
  ws.clear();
  /*ws.insert(std::pair<std::string, Eigen::Vector3f>("Workstation1",Eigen::Vector3f(-3.05,-0.95,3.14159))); //
  ws.insert(std::pair<std::string, Eigen::Vector3f>("Workstation2",Eigen::Vector3f(-4.30,-2.95,0.0)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("Workstation3",Eigen::Vector3f(-2.80,-2.65,(3.14159/2))));*/
  //verso ws1 aumenta Y; verso ws2 diminuisce Y; verso ws3 aumenta X
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-01",Eigen::Vector3f(-1.092718, -1.579077, -1.626382))); 
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-02",Eigen::Vector3f(-1.615182, -1.559591, -1.596587))); 
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-03",Eigen::Vector3f(-2.391921, 3.185186, 1.524752)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-04",Eigen::Vector3f(-1.843098, 3.166767, 1.534059)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-05",Eigen::Vector3f(0.187819, 1.077987, 0.007680)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-06",Eigen::Vector3f(0.189432, 0.806119, -0.009547)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-07",Eigen::Vector3f(0.184023, 0.484868, 0.018800)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-01",Eigen::Vector3f(-2.283176, -0.748615, 3.138756))); 
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-02",Eigen::Vector3f(-2.297204, -0.477632, -3.118710))); 
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-03",Eigen::Vector3f(-2.315036, -0.189135, -3.129054)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-04",Eigen::Vector3f(-2.354975, 0.151228, -3.139329)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-05",Eigen::Vector3f(-2.355564, 0.449728, -3.141372))); 
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-06",Eigen::Vector3f(-2.376518, 0.714417, -3.121222))); 
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-07",Eigen::Vector3f(-2.384502, 1.050055, -3.123620)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-08",Eigen::Vector3f(-2.378166, 1.300049, 3.134166)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-09",Eigen::Vector3f(-2.399218, 1.589414, 3.140204))); 
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-07",Eigen::Vector3f(-2.410785, 1.941518, 3.140511)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-08",Eigen::Vector3f(-2.413331, 2.207622, 3.132902)));
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-09",Eigen::Vector3f(-2.434230, 2.472541, 3.132413)));
}

void RockinPNPActionServer::read_orders_from_cfh(std::vector<std::pair<std::string, std::string> >& o)
{
  /*o.clear();
  o.push_back(std::pair<std::string, std::string>("Obj1","Workstation3"));
  o.push_back(std::pair<std::string, std::string>("Obj2","Workstation3"));
  o.push_back(std::pair<std::string, std::string>("Obj3","Workstation3"));*/
}

void RockinPNPActionServer::read_items_from_cfh(std::map<std::string, std::string>& it)
{
  /*it.clear();
  it.insert(std::pair<std::string, std::string>("Obj1","Workstation1"));
  it.insert(std::pair<std::string, std::string>("Obj2","Workstation2"));
  it.insert(std::pair<std::string, std::string>("Obj3","Workstation1"));*/
}

/*
 * ACTIONS
 */

void RockinPNPActionServer::init(string params, bool *run)
{
  //WORK WITH CFH
  ros::Rate loop_rate(10);
  cfh_data_mtx.lock();
  while(workstations.size()==0 || orders.size()==0 || items_state.items.size()==0){
    cfh_data_mtx.unlock();
    loop_rate.sleep();
    cfh_data_mtx.lock();
  }
  cfh_data_mtx.unlock();
  //WORK IN DEBUG
  /*read_workstations_locations(workstations);
  read_orders_from_cfh(orders);
  read_items_from_cfh(items_state);*/
  orders_index=0;
  grasp_flag=1;
  string param_grasp = "PNPconditionsBuffer/grasp_flag";
  handle.setParam(param_grasp, grasp_flag);
}

void RockinPNPActionServer::move(string params, bool *run) {
  ROS_INFO("move started ...");
  
  if(grasp_flag==1){
    //to do...
    //std::string obj=orders[orders_index].first;
    if(bring_biring_box){
      std::string dest;
      for (int i = 0; i < items_state.items.size(); ++i)
      {
        if((int)items_state.items[i].object.type.data == 2){
          for(int j=0;j<orders.size();j++){
            if(items_state.items[i].object.description.data == orders[j].object.description.data){
              dest = orders[j].destination.description.data;
              biring_box_loc = dest;
            }
          }
        }
      }
      Eigen::Vector3f loc; loc=workstations[dest];
      do_move(loc(0),loc(1),loc(2),run);
      bring_biring_box = false;
    }
    else{
      std::string dest;
      for (int i = 0; i < items_state.items.size(); ++i)
      {

      }
      Eigen::Vector3f loc; loc=workstations[dest];
      do_move(loc(0),loc(1),loc(2),run);
    }
  }else{
    Eigen::Vector3f loc; loc=workstations[biring_box_loc];
    do_move(loc(0),loc(1),loc(2),run);
  }
  /*float GX,GY,Gtheta;
  if (getLocationPosition(params,GX,GY,Gtheta)) {
    do_move(GX,GY,Gtheta,run);
  }
  else 
    ROS_WARN("Advertise: Cannot find location %s.",params.c_str());*/
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
    ros::Duration(5).sleep();
/*
    if (*run)
        ROS_INFO("### Finished Wait");
    else
        ROS_INFO("### Aborted Wait");
*/
}

void RockinPNPActionServer::detection(string params, bool *run) {
  counter_detection=0;
  ROS_INFO("detection started ...");
  detected_objects.poses.clear();
  if (ac_detection==NULL) { //create the client only once
    // Define the action client (true: we want to spin a thread)
    ac_detection = new actionlib::SimpleActionClient<rgbd_object_detection::DetectObjectsAction>(TOPIC_DETECTION_NODE, true);

    // Wait for the action server to come up
    while(!ac_detection->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for ac_detection action server to come up");
    }
  }
  //ROS_INFO("detection agent initialized ...");
  while(counter_detection<3)
  {
    rgbd_object_detection::DetectObjectsGoal goal;
    ac_detection->sendGoal(goal);
    ac_detection->waitForResult();
    counter_detection++;
    if(ac_detection->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      //ROS_INFO("cannot move the base for some reason (step 1)");
      string param = "PNPconditionsBuffer/objDetected";
      handle.setParam(param, 0);
    }else{
      counter_detection=0;
      rgbd_object_detection::DetectObjectsResultConstPtr result;
      result = ac_detection->getResult();

      /*std::vector<Eigen::Affine3d> poses;
      PoseArrayToEigen(result->objects,poses);*/
      detected_objects = result->objects;
      string param = "PNPconditionsBuffer/objDetected";
      handle.setParam(param, 1);
      /*for(int i=0; i<poses.size();++i){
        poses[i]=T_kinect2arm*poses[i];
        if(poses[i].translation()(0)<=0.0 && poses[i].translation()(0)>=-0.20 && poses[i].translation()(1)>=-0.4){
          detected_objects.poses.push_back(result->objects.poses[i]);
          string param = "PNPconditionsBuffer/objDetected";
          handle.setParam(param, 1);
        }
      }*/
      return;
    }
  }

}

//verifyGrasp action - for task1
void RockinPNPActionServer::verifyGrasp(string params, bool *run) {
  ROS_INFO("verifyGrasp action started ...");
  counter_detection=0;
  detected_objects.poses.clear();
  if (ac_detection==NULL) { //create the client only once
    // Define the action client (true: we want to spin a thread)
    ac_detection = new actionlib::SimpleActionClient<rgbd_object_detection::DetectObjectsAction>(TOPIC_DETECTION_NODE, true);

    // Wait for the action server to come up
    while(!ac_detection->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for ac_detection action server to come up");
    }
  }
  //ROS_INFO("detection agent initialized ...");
  ros::Rate lr(10);
  //ros::Rate sleeper(0.2);
  while(counter_detection<3 && feedbackToCFH.grasp_notification.data==false){
    rgbd_object_detection::DetectObjectsGoal goal;
    ac_detection->sendGoal(goal);
    ac_detection->waitForResult();
    counter_detection++;
    if(ac_detection->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    { 
      //ROS_INFO("cannot move the base for some reason (step 1)");
      //feedbackToCFH.phase_to_terminate.data = 2;
      //sleeper.sleep();
      sleep(4);
      //while(true){
      call_flag=false;
      for(int h=0;h<20;h++){
        feedbackToCFH.phase_to_terminate.data = 0;
        feedbackToCFH.grasp_notification.data = true;
        feedback_pub.publish(feedbackToCFH);
        lr.sleep();
      }
      call_flag=true;
      string param = "PNPconditionsBuffer/objGrasped";
      handle.setParam(param, 1);
      grasp_flag=0;
      string param_grasp = "PNPconditionsBuffer/grasp_flag";
      handle.setParam(param_grasp, grasp_flag);
    }else{
      if(counter_verify_grasp>=3){
        //feedbackToCFH.phase_to_terminate.data = 2;
        //while(true){
        call_flag=false;
        for(int h=0;h<20;h++){
          feedbackToCFH.phase_to_terminate.data = 0;
          feedbackToCFH.grasp_notification.data = false;
          feedback_pub.publish(feedbackToCFH);
          lr.sleep();
        }
        call_flag=true;
        std_msgs::String msg2; msg2.data="stop";
        plantoexec_pub.publish(msg2); 
        start_plan = true;
        stop_plan=false;
      }
      /*feedbackToCFH.phase_to_terminate.data = 2;
      feedbackToCFH.grasp_notification.data = true;
      feedback_pub.publish(feedbackToCFH);*/
      if(!feedbackToCFH.grasp_notification.data){
        counter_detection=0;
        string param = "PNPconditionsBuffer/objGrasped";
        handle.setParam(param, 0);
        grasp_flag=1;
        string param_grasp = "PNPconditionsBuffer/grasp_flag";
        handle.setParam(param_grasp, grasp_flag);
      }
      else{
        counter_detection=0;
        string param = "PNPconditionsBuffer/objGrasped";
        handle.setParam(param, 0);
      }
      return;
      /*rgbd_object_detection::DetectObjectsResultConstPtr result;
      result = ac_detection->getResult();
      detected_objects = result->objects;*/
    }
  }

}

void RockinPNPActionServer::detectPlane(string params, bool *run) {
  //to do...

  //ricordiamoci di settare la variabile targetDetected a 1 o 0 nella rete di Petri quando la detection fallisce o va bene
}

void RockinPNPActionServer::dropOnPlane(string params, bool *run){
  //to do...
}

void RockinPNPActionServer::grasp(string params, bool *run) {
  if(counter_verify_grasp>=5){
    ROS_INFO("PD ...");
    feedbackToCFH.grasp_notification.data = false;
    feedback_pub.publish(feedbackToCFH);
    std_msgs::String msg2; msg2.data="stop";
    plantoexec_pub.publish(msg2); 
    start_plan = true;
    stop_plan=false;
    return;
  }
  counter_verify_grasp++;
  ROS_INFO("grasping started ...");
  if (ac_manipulation==NULL) { //create the client only once
    // Define the action client (true: we want to spin a thread)
    ac_manipulation = new actionlib::SimpleActionClient<arm_planner::arm_planningAction>(TOPIC_GRASPING_NODE, true);

    // Wait for the action server to come up
    while(!ac_manipulation->waitForServer(ros::Duration(5.0))){
      //ROS_INFO("Waiting for move_base action server to come up");
    }
  }
  ROS_INFO("grasping client ready ...");
  arm_planner::arm_planningGoal goal;
  std::vector<Eigen::Affine3d> poses;
/*<<<<<<< HEAD
  if(detected_objects.poses.size()==0)
  {
    string param = "PNPconditionsBuffer/objGrasped";
    handle.setParam(param, 0);
    return;
  }
=======
>>>>>>> 48e2c8f2ed002fa430d30be661a89b92dfc75a76*/
  PoseArrayToEigen(detected_objects,poses);

  ROS_INFO("target transformed ...");
  if(detected_objects.poses.size()==0){
    //ROS_INFO("cannot move the base for some reason (step 1)");
    string param = "PNPconditionsBuffer/objGrasped";
    handle.setParam(param, 0);
    return;
  }
  Eigen::Affine3d target;
  //ROS_INFO("debug - line 1");
  target=T_kinect2arm*poses[0];
  //ROS_INFO("debug - line 2");
  goal.mode=1;
  //ROS_INFO("debug - line 3");
  goal.cartesian_position.x=target.translation()(0);
  //ROS_INFO("debug - line 4");
  goal.cartesian_position.y=target.translation()(1) - 0.03;
  //ROS_INFO("debug - line 5");
  goal.cartesian_position.z=target.translation()(2) + 0.01;
  //ROS_INFO("debug - line 6");
  goal.gripper_roll=1;
  //ROS_INFO("debug - line 7");
  goal.gripper_pitch=80;
  //ROS_INFO("debug - line 8");

  ROS_INFO("grasping sending goal ...");
  ac_manipulation->sendGoal(goal);
  ac_manipulation->waitForResult();

  if(ac_manipulation->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    //ROS_INFO("cannot move the base for some reason (step 1)");
    string param = "PNPconditionsBuffer/objGrasped";
    handle.setParam(param, 0);
    grasp_flag=1;
    string param_grasp = "PNPconditionsBuffer/grasp_flag";
    handle.setParam(param_grasp, grasp_flag);
  }else{
    string param = "PNPconditionsBuffer/objGrasped";
    handle.setParam(param, 1);
    grasp_flag=0;
    string param_grasp = "PNPconditionsBuffer/grasp_flag";
    handle.setParam(param_grasp, grasp_flag);
  }
}

void RockinPNPActionServer::drop(string params, bool *run) {
  ROS_INFO("drop started ...");
  if (ac_manipulation==NULL) { //create the client only once
    // Define the action client (true: we want to spin a thread)
    ac_manipulation = new actionlib::SimpleActionClient<arm_planner::arm_planningAction>(TOPIC_GRASPING_NODE, true);

    // Wait for the action server to come up
    while(!ac_manipulation->waitForServer(ros::Duration(5.0))){
      //ROS_INFO("Waiting for move_base action server to come up");
    }
  }
  ROS_INFO("dropping client ready ...");
  arm_planner::arm_planningGoal goal;
  /*std::vector<Eigen::Affine3d> poses;

  PoseArrayToEigen(detected_objects,poses);

  ROS_INFO("target transformed ...");
  if(detected_objects.poses.size()==0){
    //ROS_INFO("cannot move the base for some reason (step 1)");
    string param = "PNPconditionsBuffer/objGrasped";
    handle.setParam(param, 0);
    return;
  }*/
  Eigen::Vector3d target(0,-.5, .2);
  //ROS_INFO("debug - line 2");
  goal.mode=2;
  //ROS_INFO("debug - line 3");
  goal.cartesian_position.x=target(0);
  //ROS_INFO("debug - line 4");
  goal.cartesian_position.y=target(1);
  //ROS_INFO("debug - line 5");
  goal.cartesian_position.z=target(2);
  //ROS_INFO("debug - line 6");
  goal.gripper_roll=1;
  //ROS_INFO("debug - line 7");
  goal.gripper_pitch=80;
  //ROS_INFO("debug - line 8");

  ROS_INFO("dropping sending goal ...");
  ac_manipulation->sendGoal(goal);
  ac_manipulation->waitForResult();

  if(ac_manipulation->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    //ROS_INFO("cannot move the base for some reason (step 1)");
    string param = "PNPconditionsBuffer/objDropped";
    handle.setParam(param, 0);
    grasp_flag=0;
    string param_grasp = "PNPconditionsBuffer/grasp_flag";
    handle.setParam(param_grasp, grasp_flag);
  }else{
    string param = "PNPconditionsBuffer/objDropped";
    handle.setParam(param, 1);
    grasp_flag=1;
    string param_grasp = "PNPconditionsBuffer/grasp_flag";
    handle.setParam(param_grasp, grasp_flag);
    if(!bring_biring_box){
      tools_idx++;
      if(tools_idx>=orders.size()-1)
      {
        std_msgs::String msg; msg.data="stop";
        plantoexec_pub.publish(msg);
      }
    }
  }
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

//###################### CHF CALLBACKS ####################//
void RockinPNPActionServer::cb_cfh_inventory(const at_work_robot_example_ros::Inventory::ConstPtr& msg)
{
  ROS_INFO("cb_cfh_inventory called...");
  items_state = *msg;
  sub_cfh_inventory.shutdown();
  if(out_debug)
    for (int i = 0; i < items_state.items.size(); ++i)
    {
      std::cout<<"obj: "<<items_state.items[i].object.description.data<<" location: "<<items_state.items[i].location.description.data<<std::endl;
    }
}

void RockinPNPActionServer::cb_cfh_order(const at_work_robot_example_ros::OrderInfo::ConstPtr& msg)
{
  ROS_INFO("cb_cfh_order called...");
  for (int i = 0; i < msg->orders.size(); ++i)
  {
    if((int)msg->orders[i].quantity_requested.data>0){
      for (int j = 0; j < (int)msg->orders[i].quantity_requested.data; ++j)
      {
        orders.push_back(msg->orders[i]);
        tools_idx++;
      }
    }
    else{
      orders.push_back(msg->orders[i]);
    }
  }
  //orders = *msg;
  sub_cfh_order.shutdown();
  if(out_debug)
    for (int i = 0; i < orders.size(); ++i)
    {
      std::cout<<"obj: "<<orders[i].object.description.data<<std::endl;
    }
}

/*void RockinPNPActionServer::cb_cfh_conveyor_belt_command(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("cb_cfh_conveyor_belt_command called...");
}*/

void RockinPNPActionServer::cb_cfh_conveyor_belt_status(const at_work_robot_example_ros::TriggeredConveyorBeltStatus::ConstPtr& msg)
{
  ROS_INFO("sub_cfh_conveyor_belt_status called...");
  sub_cfh_conveyor_belt_status.shutdown();
}

void RockinPNPActionServer::cb_cfh_benchmark_state(const at_work_robot_example_ros::BenchmarkState::ConstPtr& msg)
{
  int state = (int)msg->state.data;
  int phase = (int)msg->phase.data;
  if(call_flag){
    if(state == 1 && phase == 0){
      //launch the pnp
      if(start_plan){
        counter_verify_grasp=0;
        feedbackToCFH.grasp_notification.data = false;
        //feedbackToCFH.phase_to_terminate.data = 0;
        //feedback_pub.publish(feedbackToCFH);
        std_msgs::String msg2; msg2.data="detect_and_grasp";
        plantoexec_pub.publish(msg2);
        start_plan = false;
        stop_plan=true;
      }
    }
    else if(state==4 && phase==0){
      //stop the plane
      if(stop_plan){
        counter_verify_grasp=0;
        feedbackToCFH.phase_to_terminate.data = phase;
        //feedback_pub.publish(feedbackToCFH);
        std_msgs::String msg2; msg2.data="stop";
        plantoexec_pub.publish(msg2); 
        start_plan = true;
        stop_plan=false;
      }
    }
    else if(state==3 && phase==0){
        if(stop_plan){
        counter_verify_grasp=0;
          feedbackToCFH.phase_to_terminate.data = phase;
          //feedback_pub.publish(feedbackToCFH);
          std_msgs::String msg2; msg2.data="stop";
          plantoexec_pub.publish(msg2); 
          start_plan = true;
          stop_plan=false;
        }
    }
  }
}

/*void RockinPNPActionServer::cb_drill_machine_status(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("cb_drill_machine_status called...");
}*/

/*void RockinPNPActionServer::cb_drilling_machine_command(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("cb_drilling_machine_command called...");
}*/