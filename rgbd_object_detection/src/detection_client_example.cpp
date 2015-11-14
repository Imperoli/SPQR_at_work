//ROS pkgs
#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include "rgbd_object_detection/DetectObjectsAction.h"




//TableArray class
// #include </opt/ros/indigo/include/object_recognition_msgs/TableArray.h>

using namespace std;
ros::Publisher pub;

actionlib::SimpleActionClient<rgbd_object_detection::DetectObjectsAction>* ac_detection;
geometry_msgs::PoseArray detected_objects;

int main (int argc, char** argv){

  // Initialize ROS
  ros::init (argc, argv, "object_detection_client");
  ros::NodeHandle nh("~");

  ac_detection=NULL;
  
  while(nh.ok())
  {
    char x;
    std::cout<<"type something"<<std::endl;
    std::cin>>x;
    ROS_INFO("detection started ...");
    detected_objects.poses.clear();
    if (ac_detection==NULL) { //create the client only once
      // Define the action client (true: we want to spin a thread)
      ac_detection = new actionlib::SimpleActionClient<rgbd_object_detection::DetectObjectsAction>("/detect_objects", true);

      // Wait for the action server to come up
      while(!ac_detection->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for ac_detection action server to come up");
      }
    }
    //ROS_INFO("detection agent initialized ...");
    rgbd_object_detection::DetectObjectsGoal goal;
    ac_detection->sendGoal(goal);
    ac_detection->waitForResult();

    if(ac_detection->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("detection aborted");
      //string param = "PNPconditionsBuffer/objDetected";
      //handle.setParam(param, 0);
      
    }else{
      
      rgbd_object_detection::DetectObjectsResultConstPtr result;
      result = ac_detection->getResult();
      detected_objects = result->objects;
      ROS_INFO("detection succeded");
      for(int i=0; i<detected_objects.poses.size(); i++){
        cout<<"obj"<<i<<" "<<detected_objects.poses[i].position.x<<" "<<detected_objects.poses[i].position.y<<" "<<detected_objects.poses[i].position.z<<endl;
      }
    }
  }
  
  return 0;
}
