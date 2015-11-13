#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"
#include "std_srvs/Empty.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

using namespace sensor_msgs;
using namespace message_filters;

boost::mutex active_mtx;

std::string rgb_topic, depth_topic, rgb_camera_info, depth_camera_info;
int cloud_number, cloud_width, cloud_height;

int iter=0;

cv::Mat sum_depth;
cv::Mat sum_rgb;

ros::Publisher rgb_pub;
ros::Publisher depth_pub;
ros::Publisher rgb_info_pub;
ros::Publisher depth_info_pub;

bool active=false;

void callback(const ImageConstPtr& rgb_image, const CameraInfoConstPtr& rgb_cam_info, const ImageConstPtr& depth_image, const CameraInfoConstPtr& depth_cam_info)
{
  active_mtx.lock();
  if(!active) return;
  // Get camera info
 /* Eigen::Matrix3d K_rgb = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d K_depth = Eigen::Matrix3d::Identity();
  int k = 0;
  for(int r = 0; r < 3; r++) {
    for(int c = 0; c < 3; c++, k++) {
      K_rgb(r, c) = rgb_cam_info->K[k];
      K_depth(r, c) = depth_cam_info->K[k];
    }
  }*/
  
  cv_bridge::CvImagePtr rgb = cv_bridge::toCvCopy(rgb_image, rgb_image->encoding);
  cv_bridge::CvImagePtr depth = cv_bridge::toCvCopy(depth_image, depth_image->encoding);
  //std::cerr<<depth->image.type()<<std::endl;
  for(int r = 0; r < depth->image.rows; r++) {
    for(int c = 0; c < depth->image.cols; c++) {
      if(depth->image.at<unsigned short>(r,c)==0 || sum_depth.at<unsigned short>(r,c)>0) continue;
      sum_depth.at<unsigned short>(r,c)=depth->image.at<unsigned short>(r,c);
    }
  }
  for(int r = 0; r < rgb->image.rows; r++) {
    for(int c = 0; c < rgb->image.cols; c++) {
      int c0=(((int)sum_rgb.at<cv::Vec3b>(r,c)[0]*iter) + ((int)rgb->image.at<cv::Vec3b>(r,c)[0]))/(iter+1);
      int c1=(((int)sum_rgb.at<cv::Vec3b>(r,c)[1]*iter) + ((int)rgb->image.at<cv::Vec3b>(r,c)[1]))/(iter+1);
      int c2=(((int)sum_rgb.at<cv::Vec3b>(r,c)[2]*iter) + ((int)rgb->image.at<cv::Vec3b>(r,c)[2]))/(iter+1);
      cv::Vec3b color(c0,c1,c2);
      sum_rgb.at<cv::Vec3b>(r,c)=color;
      //std::cerr<<rgb->image.at<cv::Vec3b>(r,c)<<", "<<sum_rgb.at<cv::Vec3b>(r,c)<<std::endl;
      rgb->image.at<cv::Vec3b>(r,c)=sum_rgb.at<cv::Vec3b>(r,c);
    }
  }
  iter++;
  if(iter>=cloud_number)
  {
    
    for(int r = 0; r < depth->image.rows; r++) {
      for(int c = 0; c < depth->image.cols; c++) {
        depth->image.at<unsigned short>(r,c)=sum_depth.at<unsigned short>(r,c);
      }
    }
  
    rgb_pub.publish(rgb->toImageMsg());
    depth_pub.publish(depth->toImageMsg());
    
    CameraInfo rgb_info, depth_info;
    rgb_info=*rgb_cam_info; depth_info=*depth_cam_info;
    rgb_info_pub.publish(rgb_info);
    depth_info_pub.publish(depth_info);
  
    iter=0;
    sum_depth=cv::Mat::zeros(cloud_height,cloud_width,CV_16UC1);
    sum_rgb=cv::Mat::zeros(cloud_height,cloud_width,CV_8UC3);
    active=false;
  }
  active_mtx.unlock();
}

bool srv_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  active_mtx.lock();
  if(!active)
    active=true;
  active_mtx.unlock();
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_accumulator");

  ros::NodeHandle nh("~");

  nh.param("rgb_image_topic",rgb_topic,std::string("/kinect/rgb/image_color"));
  nh.param("depth_image_topic",depth_topic,std::string("/kinect/depth_registered/image_raw"));
  nh.param("rgb_camera_info_topic",rgb_camera_info,std::string("/kinect/rgb/camera_info"));
  nh.param("depth_camera_info_topic",depth_camera_info,std::string("/kinect/depth_registered/camera_info"));
  nh.param("cloud_number",cloud_number,50);
  nh.param("cloud_width",cloud_width,640);
  nh.param("cloud_height",cloud_height,480);
  
  std::cerr<<"rgb_image_topic: "<<rgb_topic<<std::endl;
  std::cerr<<"depth_image_topic: "<<depth_topic<<std::endl;
  std::cerr<<"rgb_camera_info_topic: "<<rgb_camera_info<<std::endl;
  std::cerr<<"depth_camera_info_topic: "<<depth_camera_info<<std::endl;
  std::cerr<<"cloud_number: "<<cloud_number<<std::endl;
  std::cerr<<"cloud_width: "<<cloud_width<<std::endl;
  std::cerr<<"cloud_height: "<<cloud_height<<std::endl;
  
  rgb_pub = nh.advertise<sensor_msgs::Image>("rgb/image_raw", 1);
  depth_pub = nh.advertise<sensor_msgs::Image>("depth_registered/image_raw", 1);
  rgb_info_pub = nh.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1);
  depth_info_pub = nh.advertise<sensor_msgs::CameraInfo>("depth_registered/camera_info", 1);

  message_filters::Subscriber<Image> rgb_sub(nh, rgb_topic, 1);
  message_filters::Subscriber<Image> depth_sub(nh, depth_topic, 1);
  message_filters::Subscriber<CameraInfo> rgb_camera_info_sub(nh, rgb_camera_info, 1);
  message_filters::Subscriber<CameraInfo> depth_camera_info_sub(nh, depth_camera_info, 1);

  typedef sync_policies::ApproximateTime<Image, CameraInfo, Image, CameraInfo> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, rgb_camera_info_sub, depth_sub, depth_camera_info_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  
  sum_depth=cv::Mat(cloud_height,cloud_width,CV_16UC1,cv::Scalar(0));
  sum_rgb=cv::Mat(cloud_height,cloud_width,CV_8UC3,cv::Scalar(0,0,0));

  ros::ServiceServer accumulate_srv=nh.advertiseService("accumulate_clouds", srv_cb);

  ros::spin();

  return 0;
}
