#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "object_recognition_msgs/TableArray.h"
#include "object_recognition_msgs/Table.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "rgbd_object_detection/DetectObjectsAction.h"
#include <actionlib/server/simple_action_server.h>

#include "std_srvs/Empty.h"

using namespace message_filters;

std::string clusters_topic, tables_topic, rgb_optical_frame, arm_base_frame;
double table_z_offset, table_max_z, table_inclination_threshold, max_object_z_displacement; 

ros::Publisher obj_pub;
ros::Publisher tables_pub;

Eigen::Affine3d T_kinect_arm;

bool flag_clusters, flag_tables;

boost::mutex clusters_mtx, tables_mtx;

visualization_msgs::MarkerArray clusters;
object_recognition_msgs::TableArray tables;

actionlib::SimpleActionServer<rgbd_object_detection::DetectObjectsAction>* as;
ros::ServiceClient srv_start_accumulate_client, srv_stop_accumulate_client;

void tf2Affine(tf::StampedTransform& tf, Eigen::Affine3d& T)
{
  tf::Vector3 o=tf.getOrigin();
  tf::Quaternion q_tf=tf.getRotation();
  Eigen::Quaterniond q(q_tf[3],q_tf[0],q_tf[1],q_tf[2]);
  Eigen::Matrix3d R(q);
  Eigen::Vector3d t(o[0],o[1],o[2]);
  T.linear()=R; T.translation()=t;
  
}

bool computeTransformation(std::string target, std::string source, Eigen::Affine3d& T)
{
  tf::TransformListener listener;
	tf::StampedTransform transform;
	for (int i=0; i<3;i++)
	{
	  try
    {
      ros::Time now=ros::Time::now();
      listener.waitForTransform( target, source, now, ros::Duration(.25));
      listener.lookupTransform( target, source, now, transform);
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

bool computeClustersCentroids(const visualization_msgs::MarkerArray msg, geometry_msgs::PoseArray& poses)
{
   //poses.poses.clear(); // Clear last block perception result
  poses.header.stamp = ros::Time::now();
 // poses.header.frame_id = "/camera_depth_frame";
  if (msg.markers.size()==0) return false;
  poses.header.frame_id = msg.markers[0].header.frame_id;


  geometry_msgs::Pose centroid;
  centroid.orientation.x=0;
  centroid.orientation.y=0;
  centroid.orientation.z=0;
  centroid.orientation.w=1;

   double centroid_x = 0;
   double centroid_y = 0;
   double centroid_z = 0;
   

   for (unsigned int i = 0; i < msg.markers.size(); i ++)
   {
 
     visualization_msgs::Marker marker = msg.markers[i];
     geometry_msgs::Pose pose;
     pose = msg.markers[i].pose;

     double sum_x = 0;
     double sum_y = 0;
     double sum_z = 0;

     for(unsigned int j = 0; j < marker.points.size(); ++j){

      geometry_msgs::Point p = marker.points[j]; 
      sum_x += p.x;
      sum_y += p.y;
      sum_z += p.z;
     }

     centroid_x = sum_x / (double)marker.points.size();
     centroid_y = sum_y / (double)marker.points.size();
     centroid_z = sum_z / (double)marker.points.size();

     centroid.position.x = centroid_x;
     centroid.position.y = centroid_y;
     centroid.position.z = centroid_z;
     
     poses.poses.push_back(centroid);
     
   }
   
   if(poses.poses.size()>0) return true;
   return false;
}

bool extractWorkspaceTable(const object_recognition_msgs::TableArray tables, int& table_index)
{

  table_index=-1;
  for (int i=0; i<tables.tables.size(); i++){
    Eigen::Affine3d T;
    tf::poseMsgToEigen (tables.tables[i].pose, T);
    T=T_kinect_arm*T;
    Eigen::Vector3d z=Eigen::Vector3d(0,0,1);
    if (T.linear().col(2).dot(z)<table_inclination_threshold||(T.translation()(2)<(table_z_offset-0.02)||T.translation()(2)>(table_z_offset+table_max_z))) continue;
    table_index=i;
    break;
  }
  
  if(table_index!=-1){
    object_recognition_msgs::TableArray tabl;
    tabl.header=tables.header;
    tabl.tables.push_back(tables.tables[table_index]);
    tables_pub.publish(tabl);
    //ros::spinOnce();
    return true;
  }
  return false;
} 

bool extractTabletopObjects(const object_recognition_msgs::Table& table, const geometry_msgs::PoseArray& centroids, geometry_msgs::PoseArray& tabletop_objects)
{
  tabletop_objects.poses.clear();
  tabletop_objects.header=centroids.header;
  
  Eigen::Affine3d T;
  tf::poseMsgToEigen (table.pose, T);
  //T=T_kinect_arm*T;
  
  std::vector<Eigen::Vector3d> poly;
  for (int i=0; i<table.convex_hull.size(); i++){
    Eigen::Vector3d p(table.convex_hull[i].x,table.convex_hull[i].y,table.convex_hull[i].z);
    //p=T*p;
    poly.push_back(p);
  }
  
  for (int i=0; i<centroids.poses.size(); i++){
    
    Eigen::Affine3d O;
    tf::poseMsgToEigen (centroids.poses[i], O);
    //O=T_kinect_arm*O;
    O=T.inverse()*O;
    Eigen::Vector3d obj=O.translation();
    //std::cout<<"obj: "<<obj.transpose()<<std::endl;
    
    
    //if(obj(2)-T.translation()(2)>.2||obj(2)-T.translation()(2)<0) continue;
    if(obj(2)>max_object_z_displacement||obj(2)<0) continue;
    
    std::vector<Eigen::Vector2d> poly_translated;
    Eigen::Vector2d o=obj.head(2);
    for (int j=0; j<poly.size(); j++){
      Eigen::Vector2d p=poly[j].head(2)-o; 
      poly_translated.push_back(p);
      //std::cout<<"p: "<<p.transpose()<<" "<<poly[j](2)<<std::endl;
    }
    
    bool segno, flag, first;
    flag=true;
    first=true;
    for (int j=0; j<poly_translated.size(); j++){
      int j1=j+1;
      if (j1==poly_translated.size()) j1=0;
      if((poly_translated[j1]-poly_translated[j]).norm()<.015) continue;
      double a=poly_translated[j1](0)*poly_translated[j](1) - poly_translated[j](0)*poly_translated[j1](1);
      if(first)
      {
        (a>0)? segno=true:segno=false;
        first=false;
        continue;
      }
      if((a>0&&segno)||(a<=0&&!segno))
      {
        //std::cout<<"dajeeee "<<i<<" "<<j<<" "<<table.convex_hull.size()<<std::endl;
        //objects_array.poses.push_back(centroids.poses[i]);
      }
      else
      {
        flag=false;
        break;
      }
    }
    
    if(flag)
    {
      geometry_msgs::Pose p=centroids.poses[i];
      p.orientation=table.pose.orientation;
      tabletop_objects.poses.push_back(p);
    }
  }

  return (tabletop_objects.poses.size()>0);
}

bool callback(const object_recognition_msgs::TableArray tables, const visualization_msgs::MarkerArray clusters, geometry_msgs::PoseArray& out)
{
 // ROS_INFO("starting compute centroids");
  geometry_msgs::PoseArray centroids;
  if(!computeClustersCentroids(clusters, centroids)) return false;
  
 // ROS_INFO("starting extract wstable");
  int table_index=-1;
  if(!extractWorkspaceTable(tables, table_index)) return false;
  
  //ROS_INFO("starting extract tabletop objects");
  geometry_msgs::PoseArray tabletop_objects;
  if(extractTabletopObjects(tables.tables[table_index], centroids, tabletop_objects))
  {
    obj_pub.publish(tabletop_objects);
    out=tabletop_objects;
    return true;
  }
    
  return false;
}

void tables_cb(const object_recognition_msgs::TableArray::ConstPtr msg)
{
  tables_mtx.lock();
  tables=*msg;
  flag_tables=true;
  tables_mtx.unlock();
}

void clusters_cb(const visualization_msgs::MarkerArray::ConstPtr msg)
{
  clusters_mtx.lock();
  clusters=*msg;
  flag_clusters=true;
  clusters_mtx.unlock();
}

void action_cb(const rgbd_object_detection::DetectObjectsGoalConstPtr& goal)
{
  ROS_INFO("object detection action started");
  rgbd_object_detection::DetectObjectsResult result;
  std_srvs::Empty srv;
  if(!srv_start_accumulate_client.call(srv))
  {
    ROS_INFO("cannot start accumulate_cloud srv");
    as->setAborted(result);
    return;
  }

  ROS_INFO("waiting for clusters and tables");
  clusters_mtx.lock();
  tables_mtx.lock();
  while(!flag_tables||!flag_clusters){
    tables_mtx.unlock();
    clusters_mtx.unlock();

    //ros::spinOnce();
    usleep(500000); 
    //ROS_INFO("waiting for clusters and tables (cycle)");
    
    clusters_mtx.lock();
    tables_mtx.lock();
  }

  //ROS_INFO("clusters and tables ACQUIRED");
  flag_tables=false;
  flag_clusters=false;
  std_srvs::Empty srv2;
  if(!srv_stop_accumulate_client.call(srv2))
  {
    ROS_INFO("cannot stop accumulate_cloud srv");
    as->setAborted(result);
    return;
  }
  //ROS_INFO("accumulate clouds stopped");
  //clusters_mtx.lock();
  //tables_mtx.lock();
  
  geometry_msgs::PoseArray tabletop_objects;
  if(callback(tables, clusters, tabletop_objects))
  {
    result.objects=tabletop_objects;
    as->setSucceeded(result);
    tables_mtx.unlock();
    clusters_mtx.unlock();
    ROS_INFO("object detection action ended OK");
    return;
  }
  
  as->setAborted(result);
  tables_mtx.unlock();
  clusters_mtx.unlock();
  ROS_INFO("object detection action ended FAIL");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "oject_detection_pipeline");

  ros::NodeHandle nh("~");

  nh.param("clusters_topic",clusters_topic,std::string("/tabletop/clusters"));
  nh.param("tables_topic",tables_topic,std::string("/table_array"));
  nh.param("rgb_optical_frame",rgb_optical_frame,std::string("kinect_rgb_optical_frame"));
  nh.param("arm_base_frame",arm_base_frame,std::string("arm_base"));
  nh.param("table_z_offset",table_z_offset,(double) 0.0); // [m] w.r.t. arm_base reference frame
  nh.param("table_max_z",table_max_z,0.2);
  nh.param("table_inclination_threshold",table_inclination_threshold,0.9); // between 0 and 1: 1 means that the table normal has to be parallel to the z axis of the arm_base frame
  nh.param("max_object_z_displacement",max_object_z_displacement,0.2); // [m] from the table
  
  std::cerr<<"object detection action server"<<std::endl;
  std::cerr<<"cluster_topic: "<<clusters_topic<<std::endl;
  std::cerr<<"tables_topic: "<<tables_topic<<std::endl;
  std::cerr<<"rgb_optical_frame: "<<rgb_optical_frame<<std::endl;
  std::cerr<<"arm_base_frame: "<<arm_base_frame<<std::endl;
  std::cerr<<"table_z_offset: "<<table_z_offset<<std::endl;
  std::cerr<<"table_max_z: "<<table_max_z<<std::endl;
  std::cerr<<"table_inclination_threshold: "<<table_inclination_threshold<<std::endl;
  std::cerr<<"max_object_z_displacement: "<<max_object_z_displacement<<std::endl;
  
  obj_pub = nh.advertise<geometry_msgs::PoseArray> ("objects_centroid", 1);
  tables_pub = nh.advertise<object_recognition_msgs::TableArray> ("workspace_table", 1);
  
  ros::Subscriber clusters_sub = nh.subscribe(clusters_topic, 1, clusters_cb);
  ros::Subscriber tables_sub = nh.subscribe(tables_topic, 1, tables_cb);

  computeTransformation(arm_base_frame, rgb_optical_frame, T_kinect_arm);
  
  flag_clusters=false; flag_tables=false;
  
  srv_start_accumulate_client = nh.serviceClient<std_srvs::Empty>("/accumulate_clouds_start");
  srv_stop_accumulate_client = nh.serviceClient<std_srvs::Empty>("/accumulate_clouds_stop");
  
  as =new actionlib::SimpleActionServer<rgbd_object_detection::DetectObjectsAction>(nh, "/detect_objects", action_cb, false);
  as->start();
  
  ros::spin();

  return 0;
}
