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

std::string clusters_topic, tables_topic;

ros::Publisher obj_pub;
ros::Publisher tables_pub;

Eigen::Affine3d T_kinect_arm;

bool flag_clusters, flag_tables;

boost::mutex clusters_mtx, tables_mtx;

visualization_msgs::MarkerArray clusters;
object_recognition_msgs::TableArray tables;

actionlib::SimpleActionServer<rgbd_object_detection::DetectObjectsAction>* as;
ros::ServiceClient srv_client;

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
    if (T.linear().col(2).dot(z)<.9||(T.translation()(2)<0||T.translation()(2)>.2)) continue;
    table_index=i;
    break;
  }
  
  if(table_index!=-1){
    object_recognition_msgs::TableArray tabl;
    tabl.header=tables.header;
    tabl.tables.push_back(tables.tables[table_index]);
    tables_pub.publish(tabl);
    ros::spinOnce();
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
    if(obj(2)>.2||obj(2)<0) continue;
    
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
      if((poly_translated[j1]-poly_translated[j]).norm()<.01) continue;
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
  geometry_msgs::PoseArray centroids;
  computeClustersCentroids(clusters, centroids);
  
  int table_index=-1;
  if(!extractWorkspaceTable(tables, table_index)) return false;
  
  geometry_msgs::PoseArray tabletop_objects;
  if(extractTabletopObjects(tables.tables[table_index], centroids, tabletop_objects))
  {
    //obj_pub.publish(tabletop_objects);
    out=tabletop_objects;
    return true;
  }
    
  return false;
  //ros::spinOnce();
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
  std_srvs::Empty srv;
  srv_client.call(srv);

  rgbd_object_detection::DetectObjectsResult result;
  ROS_INFO("waiting for clusters and tables");
  while(!flag_tables){
    sleep(.5);
    //ros::spinOnce();
  }
  while(!flag_clusters){
    sleep(.5);
    //ros::spinOnce();
  }
  flag_tables=false;
  flag_clusters=false;
  
  clusters_mtx.lock();
  tables_mtx.lock();
  
  geometry_msgs::PoseArray tabletop_objects;
  if(callback(tables, clusters, tabletop_objects))
  {
    result.objects=tabletop_objects;
    as->setSucceeded(result);
    tables_mtx.unlock();
    clusters_mtx.unlock();
    return;
  }
  
  as->setAborted(result);
  tables_mtx.unlock();
  clusters_mtx.unlock();
  //ROS_INFO("action_cb end");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "oject_recognition_pipeline");

  ros::NodeHandle nh("~");

  nh.param("clusters_topic",clusters_topic,std::string("/tabletop/clusters"));
  nh.param("tables_topic",tables_topic,std::string("/table_array"));
  
  std::cerr<<"rgb_image_topic: "<<clusters_topic<<std::endl;
  std::cerr<<"depth_image_topic: "<<tables_topic<<std::endl;
  
  obj_pub = nh.advertise<geometry_msgs::PoseArray> ("objects_centroid", 1);
  tables_pub = nh.advertise<object_recognition_msgs::TableArray> ("workspace_table", 1);
  
  ros::Subscriber clusters_sub = nh.subscribe(clusters_topic, 1, clusters_cb);
  ros::Subscriber tables_sub = nh.subscribe(tables_topic, 1, tables_cb);

  computeTransformation("arm_base", "kinect_rgb_optical_frame", T_kinect_arm);
  std::cout<<"T_kinect2arm:\n"<<T_kinect_arm.matrix()<<std::endl;
  
  flag_clusters=false; flag_tables=false;
  
  srv_client = nh.serviceClient<std_srvs::Empty>("accumulate_clouds");
  
  as =new actionlib::SimpleActionServer<rgbd_object_detection::DetectObjectsAction>(nh, "detect_objects", action_cb, false);
  //ROS_INFO("#############  detect_objects action server started  ######################################################");
  as->start();
  
  ros::spin();
  
 /* 
  
  while(ros::ok())
  {
    ROS_INFO("waiting for clusters and tables");
    while(!flag_tables){
      sleep(.5);
      ros::spinOnce();
    }
    while(!flag_clusters){
      sleep(.5);
      ros::spinOnce();
    }
    flag_tables=false;
    flag_clusters=false;
    
    clusters_mtx.lock();
    tables_mtx.lock();
    
    callback(tables, clusters);
    
    tables_mtx.unlock();
    clusters_mtx.unlock();
    
  }*/

  return 0;
}
