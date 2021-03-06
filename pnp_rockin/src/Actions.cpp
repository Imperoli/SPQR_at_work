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
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-01",Eigen::Vector3f(-12.308,-13.303,0))); //OK sx-drilling p[-12.308;-13.303;0]o[0;0;0.013;0.999]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-02",Eigen::Vector3f(-12.954,-13.314,0))); //OK dx-drilling p[-12.954;-13.314;0]o[0;0;0.020;0.999]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-03",Eigen::Vector3f(-13.599,-8.372,(-3.1415)))); //OK sx-fitting p[-13.599;-8.372;0]o[0;0;0.999;-0.038] 
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-04",Eigen::Vector3f(-13.061,-8.349,(-3.1415)))); //OK dx-fitting p[-13.061;-8.349;0]o[0;0;0.998;-0.049]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-05",Eigen::Vector3f(-10.96,-10.66,(3.1415/2)))); //OK sx-central p[-10.96;-10.66;0]o[0;0;0.729;0.683]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-06",Eigen::Vector3f(-10.949,-10.91,(3.1415/2)))); //OK cr-central p[-10.949;-10.91;0]o[0;0;0.723;0.690]      
  ws.insert(std::pair<std::string, Eigen::Vector3f>("WORKSTATION-07",Eigen::Vector3f(-10.938,-11.165,(3.1415/2)))); //OK dx-central p[-10.938;-11.165;0]o[0;0;0.727;0.685]      
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-01",Eigen::Vector3f(-13.661,-12.325,(-3.1415/2)))); //p[-13.661;-12.325;0]o[0;0;-0.691;0.722]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-02",Eigen::Vector3f(-13.675,-12.028,(-3.1415/2)))); //p[-13.675;-12.028;0]o[0;0;-0.678;0.734]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-03",Eigen::Vector3f(-13.704,-11.789,(-3.1415/2)))); //p[-13.704;-11.789;0]o[0;0;-0.683;0.730]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-04",Eigen::Vector3f(-13.766,-11.428,(-3.1415/2)))); //p[-13.766;-11.428;0]o[0;0;-0.683;0.730]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-05",Eigen::Vector3f(-13.786,-11.204,(-3.1415/2)))); //p[-13.786;-11.204;0]o[0;0;-0.680;0.733]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-06",Eigen::Vector3f(-13.775,-10.946,(-3.1415/2)))); //p[-13.775;-10.946;0]o[0;0;-0.669;0.742]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-07",Eigen::Vector3f(-13.761,-10.594,(-3.1415/2)))); //p[-13.811;-10.594;0]o[0;0;-0.671;0.741]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-08",Eigen::Vector3f(-13.814,-10.345,(-3.1415/2)))); //p[-13.814;-10.345;0]o[0;0;-0.675;0.737]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-09",Eigen::Vector3f(-13.820,-10.077,(-3.1415/2)))); //p[-13.820;-10.077;0]o[0;0;-0.667;0.744]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-10",Eigen::Vector3f(-13.835,-9.722,(-3.1415/2)))); //p[-13.835;-9.722;0]o[0;0;-0.670;0.741]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-11",Eigen::Vector3f(-13.858,-9.474,(-3.1415/2)))); //p[-13.858;-9.474;0]o[0;0;-0.666;0.745]
  ws.insert(std::pair<std::string, Eigen::Vector3f>("SHELF-12",Eigen::Vector3f(-13.911,-9.195,(-3.1415/2)))); //p[-13.911;-9.195;0]o[0;0;-0.664;0.747]
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
  
  /*cfh_data_mtx.lock();
  
  while(workstations.size()==0 || orders.size()==0 || items_state.items.size()==0){
  	ROS_INFO("initialization");
    cfh_data_mtx.unlock();
    loop_rate.sleep();
    cfh_data_mtx.lock();
    //break;
  }
  cfh_data_mtx.unlock();*/
  //WORK IN DEBUG
  read_workstations_locations(workstations);
  /*read_orders_from_cfh(orders);
  read_items_from_cfh(items_state);*/

  order_vector.clear();
  order_vector.push_back(container_location);
  for(int i=0; i<objects_location.size(); i++){
  	order_vector.push_back(objects_location[i]);
  	std::cout<<order_vector[i]<<std::endl;
  	std::cout<<i<<std::endl;
  }
  std::cout<<order_vector.size()<<std::endl;
  std::cout<<objects_location.size()<<std::endl;

  ROS_INFO("initialized");
  order_index=0;
  grasp_flag=1;
  string param_grasp = "PNPconditionsBuffer/grasp_flag";
  handle.setParam(param_grasp, grasp_flag);
}

void RockinPNPActionServer::move(string params, bool *run) {
  ROS_INFO("move started ...");
  //std::cout<<"order_index: "<<order_index<<std::endl;
  //std::cout<<"grasp_flag: "<<grasp_flag<<std::endl;
  if(grasp_flag==1){

	  Eigen::Vector3f loc; loc=workstations[order_vector[order_index]];
	  do_move(loc(0),loc(1),loc(2),run);	//DEBUG
	  //string param = "PNPconditionsBuffer/goalReached"; //DEBUG
	  //handle.setParam(param, 1); //DEBUG
	  //std::cout<<"Moving to: "<<order_vector[order_index]<<std::endl;

    //grasp_flag=0;	//DEBUG
  }else{
  	std::cout<<"Moving to: "<<container_destination<<std::endl;
    Eigen::Vector3f loc; loc=workstations[container_destination];
    do_move(loc(0),loc(1),loc(2),run); //DEBUG
    //string param = "PNPconditionsBuffer/goalReached"; //DEBUG
    //handle.setParam(param, 1); //DEBUG
    //grasp_flag=1;	//DEBUG
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
   // if(grasp_flag)
    //	order_index++;
    
    
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

      std::vector<Eigen::Affine3d> poses;
      PoseArrayToEigen(result->objects,poses);
      //detected_objects = result->objects;
      for(int i=0; i<poses.size();++i){
        poses[i]=T_kinect2arm*poses[i];
        if(poses[i].translation()(0)<=0.0 && poses[i].translation()(0)>=-0.20 && poses[i].translation()(1)>=-0.45){
          detected_objects.poses.push_back(result->objects.poses[i]);
          string param = "PNPconditionsBuffer/objDetected";
          handle.setParam(param, 1);
        }
      }
      if(detected_objects.poses.size()==0)
      {
      	string param2 = "PNPconditionsBuffer/objDetected";
        handle.setParam(param2, 0);
      }
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
  while(counter_detection<3){
    rgbd_object_detection::DetectObjectsGoal goal;
    ac_detection->sendGoal(goal);
    ac_detection->waitForResult();
    counter_detection++;
    if(ac_detection->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    { 
      //ROS_INFO("cannot move the base for some reason (step 1)");
      string param = "PNPconditionsBuffer/objGrasped";
      handle.setParam(param, 1);
      grasp_flag=0;
      string param_grasp = "PNPconditionsBuffer/grasp_flag";
      handle.setParam(param_grasp, grasp_flag);
    }else{
      
      rgbd_object_detection::DetectObjectsResultConstPtr result;
      result = ac_detection->getResult();

      std::vector<Eigen::Affine3d> poses;
      PoseArrayToEigen(result->objects,poses);
      //detected_objects = result->objects;
      for(int i=0; i<poses.size();++i){
        poses[i]=T_kinect2arm*poses[i];
        if(poses[i].translation()(0)<=0.0 && poses[i].translation()(0)>=-0.20 && poses[i].translation()(1)>=-0.475){
        	counter_detection=0;
          //detected_objects.poses.push_back(result->objects.poses[i]);
          string param = "PNPconditionsBuffer/objGrasped";
          handle.setParam(param, 0);
          grasp_flag=1;
          string param_grasp = "PNPconditionsBuffer/grasp_flag";
      	  handle.setParam(param_grasp, grasp_flag);
          return;
        }
      }
	  	string param2 = "PNPconditionsBuffer/objGrasped";
	    handle.setParam(param2, 1);
	    grasp_flag=0;
      string param_grasp2 = "PNPconditionsBuffer/grasp_flag";
      handle.setParam(param_grasp2, grasp_flag);

      /*string param = "PNPconditionsBuffer/objGrasped";
      handle.setParam(param, 0);
      grasp_flag=1;
      string param_grasp = "PNPconditionsBuffer/grasp_flag";
      handle.setParam(param_grasp, grasp_flag);
      return;*/
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
  goal.cartesian_position.x=target.translation()(0) - 0.01;
  //ROS_INFO("debug - line 4");
  goal.cartesian_position.y=target.translation()(1) - 0.03;
  //ROS_INFO("debug - line 5");
  goal.cartesian_position.z=target.translation()(2) + 0.0125;
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
  Eigen::Vector3d target(-.1,-.4, .025);
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
	  order_index++;
	  if(order_index>=order_vector.size())
	  {
	    std_msgs::String msg; msg.data="stop";
	    plantoexec_pub.publish(msg);
	  }
  }
}

void RockinPNPActionServer::do_move(float GX, float GY, float GTh_RAD, bool *run) { // theta in degrees
  ROS_INFO("do_move started");
  //mtx_movebase.lock();

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
  /*ac_move->cancelAllGoals(); ros::Duration(0.2).sleep(); // wait a little

  mtx_movebase.unlock();*/
}

//###################### CHF CALLBACKS ####################//
void RockinPNPActionServer::cb_cfh_inventory(const at_work_robot_example_ros::Inventory::ConstPtr& msg)
{
  ROS_INFO("cb_cfh_inventory called...");
  
  	if(msg->items.size()>0){
  		ROS_INFO("items size > 0");
  		objects_location.clear();
	  //items_state = *msg;
  		for(int i=0; i<msg->items.size(); i++){
  			if((int)msg->items[i].object.type.data==1)
  				objects_location.push_back(msg->items[i].location.description.data);
  			else
  				container_location=msg->items[i].location.description.data;
  		}
  		if(!start_plan)
	    	sub_cfh_inventory.shutdown();
	}
	ROS_INFO("debug line zzzzz");
  if(out_debug)
    for (int i = 0; i < objects_location.size(); ++i)
    {
      std::cout<<" location: "<<objects_location[i]<<std::endl;
    }
}

void RockinPNPActionServer::cb_cfh_order(const at_work_robot_example_ros::OrderInfo::ConstPtr& msg)
{
  ROS_INFO("cb_cfh_order called...");
  if(msg->orders.size()>0){
	  for (int i = 0; i < msg->orders.size(); ++i)
	  {
	    if((int)msg->orders[i].object.type.data>1){
	    	container_destination=msg->orders[i].destination.description.data;
	    }
	  }
	  //orders = *msg;
	  if(!start_plan)
	  	sub_cfh_order.shutdown();
	  /*if(out_debug)
	    for (int i = 0; i < orders.size(); ++i)
	    {
	      std::cout<<"obj: "<<orders[i].object.description.data<<" destination: "<<orders[i].destination.description.data<<std::endl;
	    }*/
  }
}

/*void RockinPNPActionServer::cb_cfh_conveyor_belt_command(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("cb_cfh_conveyor_belt_command called...");
}*/

/*void RockinPNPActionServer::cb_cfh_conveyor_belt_status(const at_work_robot_example_ros::TriggeredConveyorBeltStatus::ConstPtr& msg)
{
  ROS_INFO("sub_cfh_conveyor_belt_status called...");
  sub_cfh_conveyor_belt_status.shutdown();
}*/

void RockinPNPActionServer::cb_cfh_benchmark_state(const at_work_robot_example_ros::BenchmarkState::ConstPtr& msg)
{
	int phase = (int)msg->phase.data;
	int state = (int)msg->state.data;
	if(phase == at_work_robot_example_ros::BenchmarkState::EXECUTION && state == at_work_robot_example_ros::BenchmarkState::RUNNING && start_plan){
		std_msgs::String msg2; msg2.data="task1";
        plantoexec_pub.publish(msg2);
        ROS_INFO("plan_started");
        start_plan=false;
        stop_plan=true;
	}
	if(phase == at_work_robot_example_ros::BenchmarkState::EXECUTION && state == at_work_robot_example_ros::BenchmarkState::STOPPED && stop_plan){	
		std_msgs::String msg2; msg2.data="stop";
        plantoexec_pub.publish(msg2);
        ROS_INFO("plan_stopped");
        stop_plan=false;
        start_plan=true;
        sub_cfh_inventory = handle.subscribe(TOPIC_CFH_INVENTORY, 1, &RockinPNPActionServer::cb_cfh_inventory, this);
    	sub_cfh_order = handle.subscribe(TOPIC_CFH_ORDER, 1, &RockinPNPActionServer::cb_cfh_order, this);
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