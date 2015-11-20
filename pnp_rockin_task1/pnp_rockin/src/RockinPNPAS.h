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

//CHF headers
#include <at_work_robot_example_ros/AttentionMessage.h>
#include <at_work_robot_example_ros/BenchmarkState.h>
#include <at_work_robot_example_ros/TriggeredConveyorBeltStatus.h>
#include <at_work_robot_example_ros/DrillingMachineStatus.h>
#include <at_work_robot_example_ros/Inventory.h>
#include <at_work_robot_example_ros/OrderInfo.h>
#include <at_work_robot_example_ros/BenchmarkFeedback.h>
#include <at_work_robot_example_ros/TriggeredConveyorBeltCommand.h>
#include <at_work_robot_example_ros/DrillingMachineCommand.h>
#include <at_work_robot_example_ros/LoggingStatus.h>
#include <at_work_robot_example_ros/Transaction.h>
#include <at_work_robot_example_ros/RobotStatusReport.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class RockinPNPActionServer : public PNPActionServer
{
private:
    ros::NodeHandle handle, handlep;
    ros::Publisher event_pub, plantoexec_pub, hri_pub, rcom_pub;
    tf::TransformListener* listener;

    //cfh subscribers
    ros::Subscriber sub_cfh_inventory;
    ros::Subscriber sub_cfh_order;
    //ros::Subscriber sub_cfh_conveyor_belt_command;
    ros::Subscriber sub_cfh_conveyor_belt_status;
    //ros::Subscriber sub_cfh_drill_machine_status;
    //ros::Subscriber sub_cfh_drilling_machine_command;

    // action clients
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_move;         //the action client for the movement planner
    actionlib::SimpleActionClient<rgbd_object_detection::DetectObjectsAction> *ac_detection;    //the action client for the detection phase
    actionlib::SimpleActionClient<arm_planner::arm_planningAction> *ac_manipulation;    //the action client for the grasping phase

    ros::Publisher PNP_cond_pub;
    
    boost::mutex mtx_movebase;
    boost::mutex cfh_data_mtx;

    //detected objects array
    geometry_msgs::PoseArray detected_objects;

    Eigen::Affine3d T_kinect2arm;
    at_work_robot_example_ros::Inventory items_state;
    std::map<std::string, Eigen::Vector3f> workstations;
    std::vector<at_work_robot_example_ros::Order> orders;
    int orders_index;
    int grasp_flag;
    int counter_detection;
    bool out_debug;
    bool bring_biring_box;
    std::string biring_box_loc;
    int tools_idx;

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
    void read_workstations_locations(std::map<std::string, Eigen::Vector3f>& ws);
    void read_orders_from_cfh(std::vector<std::pair<std::string, std::string> >& o);
    void read_items_from_cfh(std::map<std::string, std::string>& it);


    /*
     * ACTIONS
     */
    void wait(string params, bool *run);
    void move(string params, bool *run);
    void detection(string params, bool *run);
    void grasp(string params, bool *run);
    void drop(string params, bool *run);
    void init(string params, bool *run);
    void verifyGrasp(string params, bool *run);
    void detectPlane(string params, bool *run);
    void dropOnPlane(string params, bool *run);

    void do_move(float GX, float GY, float GTh_RAD, bool *run);

    
    /*
     * CONDITIONS CALLBACKS
     */

    /*
     * CFH CALLBACKS
     */
    void cb_cfh_inventory(const at_work_robot_example_ros::Inventory::ConstPtr& msg);
    void cb_cfh_order(const at_work_robot_example_ros::OrderInfo::ConstPtr& msg);
    void cb_cfh_conveyor_belt_status(const at_work_robot_example_ros::TriggeredConveyorBeltStatus::ConstPtr& msg);
};

#endif
