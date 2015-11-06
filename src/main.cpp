//ROS pkgs
#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



// My inclusion
// #include "detection.cpp"
//Filesystem
#include <dirent.h>


//TableArray class
// #include </opt/ros/indigo/include/object_recognition_msgs/TableArray.h>

using namespace std;
ros::Publisher pub;

//Marker Attributes
// int32 type                         # Type of object
// int32 action                         # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
// geometry_msgs/Pose pose                 # Pose of the object
// geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
// std_msgs/ColorRGBA color             # Color [0.0-1.0]
// duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
// bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

// #Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
// geometry_msgs/Point[] points

void 
clusters_callback(const visualization_msgs::MarkerArray::ConstPtr msg){
  geometry_msgs::PoseArray poses;
  //poses.poses.clear(); // Clear last block perception result
  poses.header.stamp = ros::Time::now();
 // poses.header.frame_id = "/camera_depth_frame";
  if (msg->markers.size()==0) return;
  poses.header.frame_id = msg->markers[0].header.frame_id;


  geometry_msgs::Pose centroid;
  centroid.orientation.x=0;
  centroid.orientation.y=0;
  centroid.orientation.z=0;
  centroid.orientation.w=1;

   double centroid_x = 0;
   double centroid_y = 0;
   double centroid_z = 0;
   

   for (unsigned int i = 0; i < msg->markers.size(); i ++)
   {
 
     visualization_msgs::Marker marker = msg->markers[i];
     unsigned int clusterID = marker.id;
     geometry_msgs::Pose pose;
     pose = msg->markers[i].pose;

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

   pub.publish(poses);
   std::cerr<<"published"<<endl;

}


//Table Public Attributes
// geometry_msgs::Point *  convex_hull
// uint8_t   convex_hull_length
// std_msgs::Header  header
// geometry_msgs::Pose   pose
// geometry_msgs::Point  st_convex_hull

//void 
//tables_callback(const object_recognition_msgs::TableArray::ConstPtr msg)
//{
  // while(msg->markers.size() != 0){
  //  cout << "number of tables in the scene " << msg->tables.size() << endl;
  // }

//}

int main (int argc, char** argv){

  // Initialize ROS
  ros::init (argc, argv, "object_detection_module");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_clustr = nh.subscribe<visualization_msgs::MarkerArray>("tabletop/clusters", 1, clusters_callback);
  //ros::Subscriber sub_tables = nh.subscribe<object_recognition_msgs::TableArray>("table_array", 1, tables_callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<geometry_msgs::PoseArray> ("clusters_centroid", 1);

  // Spin
  //Single thread
   ros::spin();

}