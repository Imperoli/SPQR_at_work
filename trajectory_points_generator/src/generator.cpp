#include <ros/ros.h>
#include "../include/trajectory_points_generator/params.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Point.h"
#include <math.h>


ros::Publisher points_pub;

std::vector<geometry_msgs::Pose> generate_points(bool type){
  std::vector<geometry_msgs::Pose> points;
  double dt = 0.01;
  
  if(type == 0){

    for(int i = 0; i <= (int)( x /dt); i++){
      double X=(double)i*dt;
      geometry_msgs::Pose p;
      
      p.position.x =X;
      p.position.y = a*sin((M_PI*c)*X);
      p.position.z = 0;
      points.push_back(p);
      //std::cout << "sine position " << points[i].position.y << std::endl;
      
    }

  }

  if(type == 1){

    for(double i = 0; i < (int)( x /dt); i++){
      double X=(double)i*dt;
      geometry_msgs::Pose p;

      p.position.x = X;
      p.position.y = X + m*i;
      p.position.z = 0;
      points.push_back(p);
      //std::cout << "line position " << points[i].position.y << std::endl;

    }
  }

  return points;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "points_generator");

    ros::NodeHandle node;

    bool type = 1; //0 == sin, 1 == line
    std::vector<geometry_msgs::Pose> points;
    points = generate_points(type);

    points_pub = node.advertise<geometry_msgs::Pose>("path_points",100);

    for(std::vector<geometry_msgs::Pose>::iterator it = points.begin(); it != points.end(); ++it){
         geometry_msgs::Pose p = *it;
         points_pub.publish(p); 
    }   

    ros::spin();

    return 0;
}

