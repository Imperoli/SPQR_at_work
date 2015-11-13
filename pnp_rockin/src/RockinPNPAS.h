#ifndef __ROCKINPNPAS_H__
#define __ROCKINPNPAS_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <rgbd_object_detection/DetectObjectsAction.h>
#include <arm_planner/arm_planningAction.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Dense>

#include <pnp_ros/PNPActionServer.h>

#include <boost/thread/thread.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class RockinPNPActionServer : public PNPActionServer
{
private:
    ros::NodeHandle handle, handlep;
    ros::Publisher event_pub, plantoexec_pub, hri_pub, rcom_pub;
    tf::TransformListener* listener;

    // action clients
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_move;         //the action client for the movement planner
    actionlib::SimpleActionClient<rgbd_object_detection::DetectObjectsAction> *ac_detection;    //the action client for the detection phase
    actionlib::SimpleActionClient<arm_planner::arm_planningAction> *ac_grasping;    //the action client for the grasping phase

    ros::Publisher PNP_cond_pub;
    
    boost::mutex mtx_movebase;

    //detected objects array
    geometry_msgs::PoseArray detected_objects;

    Eigen::Affine3d T_kinect2arm;

public:

    RockinPNPActionServer(ros::NodeHandle n);

    virtual int evalCondition(string cond);

    // Get current robot pose
    bool getRobotPose(string robotname, double &x, double &y, double &th_rad);

    // Get coordinates of semantic location
    bool getLocationPosition(string loc, float &GX, float &GY, float &Gtheta);

    // other functions
    void PoseArrayToEigen(const geometry_msgs::PoseArray& poses, std::vector<Eigen::Affine3d>& output);
    void tf2Affine(tf::StampedTransform& tf, Eigen::Affine3d& T);
    bool computeTransformation(std::string target, std::string source, Eigen::Affine3d& T);


    /*
     * ACTIONS
     */
    void wait(string params, bool *run);
    void move(string params, bool *run);
    void detection(string params, bool *run);
    void grasp(string params, bool *run);

    void do_move(float GX, float GY, float GTh_RAD, bool *run);

    
    /*
     * CONDITIONS CALLBACKS
     */

    
};

#endif
