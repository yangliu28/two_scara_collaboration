// test program for scara motion control
// control the scara robot by publishing joint angles on corresponding topic
// one of the two independent motion planner
// all two scara robots are positive configured

// ros communication:
    // subscribe to topic "/cylinder_blocks_poses"
    // publish to topic "/scara_robot_left_rotation1_pos_cmd"
    // publish to topic "/scara_robot_left_rotation2_pos_cmd"
    // subscribe to topic "/cylinder_active_pool"
    // service client to service "/cylinder_pool_claim"
    // subscribe to topic "/scara_right_upper_boundary"
    // publish the topic "/scara_left_upper_boundary"
    // action client for gripper control, "gripper_action"
    // service client to get cylinder speed "/gazebo/get_model_state"

// motion steps decomposed:
    // 1.stand by position -> target cylinder position
    // 2.target cylinder position hovering (for cylinder grasping operation)
    // 3.target cylinder position -> drop position
    // 4.drop position -> stand by position (high speed)
// only in motion step 1, the operation may be delayed when the other robot is in operation range

// upper boundary control method:
    // each robot has a state machine contains a boolean and a number
    // the boolean indicate if the robot is at upper area of the conveyor
    // the number indicate the upper boundary the robot goes
    // each robot check the other robot's state and make change to its own


#include <ros/ros.h>
#include <math.h>
#include <two_scara_collaboration/cylinder_blocks_poses.h>
#include <std_msgs/Int8MultiArray.h>
#include <two_scara_collaboration/scara_upper_boundary.h>
#include <actionlib/client/simple_action_client.h>
#include <two_scara_collaboration/scara_gripperAction.h>
#include <gazebo_msgs/GetModelState.h>

// global variables
std::vector<double> g_cylinder_x;  // only x y coordinates matter
std::vector<double> g_cylinder_y;
std::vector<int> g_cylinder_pool;  // the active pool
two_scara_collaboration::scara_upper_boundary g_scara_right_upper_boundary;


std::vector<double> scara_left_kinematics(std::vector<double> joints) {
    // kinematics for the left scara robot
}

std::vector<double> scara_left_inverse_kinematics(std::vector<double> position) {
    // inverse kinematics for the left scara robot
    // robot is always positive configured, so only return the position configuration
}

void cylinderPosesCallback(const two_scara_collaboration::cylinder_blocks_poses& cylinder_poses_msg) {
    // update poses message in global variables
    int cylinder_quantity = cylinder_poses_msg.x.size();
    g_cylinder_x.resize(cylinder_quantity);
    g_cylinder_y.resize(cylinder_quantity);
    g_cylinder_x = cylinder_poses_msg.x;
    g_cylinder_y = cylinder_poses_msg.y;
}

void cylinderPoolCallback(const std_msgs::Int8MultiArray& cylinder_pool_msg) {
    // update the cylinder active pool
    int pool_size = cylinder_pool_msg.data.size();
    g_cylinder_pool.resize(pool_size);
    g_cylinder_pool = cylinder_pool_msg.data;
}

void scaraRightUpperBoundaryCallback(const
    two_scara_collaboration::scara_upper_boundary& upper_boundary_msg) {
    // update on upper boundary of right scara
    g_scara_right_upper_boundary = upper_boundary_msg;
}


int main(int argc, char** argv) {
    ros::inti(argc, argv, "scara_left_motion_planner");
    ros::NodeHandle nh;

    // 
    double cylinder_speed;

    // initialize subscriber to "/cylinder_blocks_poses"
    ros::Subscriber cylinder_poses_subscriber = nh.subscribe("/cylinder_blocks_poses"
        , 1, cylinderPosesCallback);
    // publish to topic "/scara_robot_left_rotation1_pos_cmd"
    ros::Publisher scara_left_r1_cmd_publisher
        = nh.advertise<std_msgs::Float64>("/scara_robot_left_rotation1_pos_cmd", 1);
    std_msgs::Float64 scara_left_r1_cmd_msg;
    // publish to topic "/scara_robot_left_rotation2_pos_cmd"
    ros::Publisher scara_left_r2_cmd_publisher
        = nh.advertise<std_msgs::Float64>("/scara_robot_left_rotation2_pos_cmd", 1);
    std_msgs::Float64 scara_left_r2_cmd_msg;
    // subscribe to topic "/cylinder_active_pool"
    ros::Subscriber cylinder_pool_subscriber = nh.subscribe("/cylinder_active_pool"
        , 1, cylinderPoolCallback);
    // service client to service "/cylinder_pool_claim"
    ros::ServiceClient cylinder_pool_claim_client
        = nh.serviceClient<two_scara_collaboration::pool_claim_msg>("cylinder_pool_claim");
    two_scara_collaboration::pool_claim_msg cylinder_claim_srv_msg;
    // subscribe to topic "/scara_right_upper_boundary"
    ros::Subscriber scara_right_upper_boundary_subscriber
        = nh.subscribe("/scara_right_upper_boundary", 1, scaraRightUpperBoundaryCallback);
    // publish on topic "/scara_left_upper_boundary"
    ros::Publisher scara_left_upper_boundary_publisher
        = nh.advertise<two_scara_collaboration::scara_upper_boundary>("/scara_left_upper_boundary", 1);
    two_scara_collaboration::scara_left_upper_boundary scara_left_upper_boundary_msg;
    // action client for gripper control, "gripper_action"
    actionlib::SimpleActionClient<two_scara_collaboration::scara_gripperAction>
        scara_gripper_action_client("gripper_action", true);
    two_scara_collaboration::scara_gripperGoal scara_gripper_goal;
    // service client to get cylinder speed "/gazebo/get_model_state"
    ros::Serviceclient get_model_state_client
        = nh.serviceClient<gazebo_msgs/GetModelState>("/gazebo/get_model_state");

    // check if service "/gazebo/get_model_state" is ready
    bool service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/get_model_state", true);
        ROS_INFO("waiting for get_model_state service");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("get_model_state service is ready");

    // get the speed of the last cylinder in queue
    // so that it has speed instead of falling out of conveyor




}



