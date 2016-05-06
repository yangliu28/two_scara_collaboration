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
    // service client to service "/scara_left_upper_boundary_change"
    // action client for gripper control, "gripper_action"

// motion steps decomposed:
    // 1.stand by position -> target cylinder position
    // 2.target cylinder hovering (for cylinder grasping operation)
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
#include <two_scara_collaboration/upper_boundary_change.h>
#include <actionlib/client/simple_action_client.h>
#include <two_scara_collaboration/scara_gripperAction.h>
#include <gazebo_msgs/GetModelState.h>

// the measured absolute speed of the cylinder
const double CYLINDER_SPEED = 1.2;  // the cylinder is moving along -x direction

// global variables
std::vector<double> g_cylinder_x;  // only x y coordinates matter
std::vector<double> g_cylinder_y;
std::vector<int> g_cylinder_pool;  // the active pool
bool g_right_in_action;  // upper boundary status of right scara
double g_right_upper_boundary;


// kinematics for the left scara robot
std::vector<double> scara_left_kinematics(std::vector<double> joints) {
    // x_generic and y_generic for the generic coordinates used in scara kinematics
    double x_generic = cos(joints[0]) + 0.8*cos(joints[0] + joints[1]);
    double y_generic = sin(joints[0]) + 0.8*sin(joints[0] + joints[1]);
    // convert to coordinates in the view of left scara
    std::vector<double> output_position;
    output_position.resize(2);
    output_position[0] = y_generic;  // x coordinate
    output_position[1] = 1.5 - x_generic;  // y coordinate
    return output_position;
}

// inverse kinematics for the left scara robot
std::vector<double> scara_left_inverse_kinematics(std::vector<double> position) {
    // the returned robot configuration is always positive
    // convert position to the generic coordinates in scara kinematics
    double x_generic = 1.5 - position[1];
    double y_generic = position[0];
    std::vector<double> output_joints;
    output_joints.resize(2);
    double end_quad_dist = pow(x_generic, 2) + pow(y_generic, 2);
    output_joints[0] = atan(y_generic/x_generic) - acos((0.36 + end_quad_dist)/(2*sqrt(end_quad_dist)));
    output_joints[1] = M_PI - acos((1.64 - end_quad_dist)/1.6);
    return output_joints;
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
    g_right_in_action = upper_boundary_msg.in_action;
    g_right_upper_boundary = upper_boundary_msg.upper_boundary;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "scara_left_motion_planner");
    ros::NodeHandle nh;

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
    // service client to service "/scara_left_upper_boundary_change"
    ros::ServiceClient scara_left_upper_boundary_client
        = nh.serviceClient<two_scara_collaboration::upper_boundary_change>("/scara_left_upper_boundary_change");
    two_scara_collaboration::upper_boundary_change left_boundary_change_srv;    
    // action client for gripper control, "gripper_action"
    actionlib::SimpleActionClient<two_scara_collaboration::scara_gripperAction>
        scara_gripper_action_client("gripper_action", true);
    two_scara_collaboration::scara_gripperGoal scara_gripper_goal;

    // use "/gazebo/get_model_state" only to check if Gazebo is up and running
    bool gazebo_ready = false;
    while (!gazebo_ready) {
        gazebo_ready = ros::service::exists("/gazebo/get_model_state", true);
        ROS_INFO("waiting for gazebo to start");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("gazebo is ready");

    // the motion planner loop
    while (ros::ok()) {
        // motion has been divided into four parts according to the purposes
        // 2.target cylinder hovering (for cylinder grasping operation)
        // 3.target cylinder position -> drop position
        // 4.drop position -> stand by position (high speed)

        // 1.stand by position -> target cylinder position
        // claim cylinders from cylinder active pool
        bool cylinder_claimed = false;
        int cylinder_index;
        while (!cylinder_claimed) {
            // cylinder not claimed
            for (int i=0; i<g_cylinder_pool.size(); i++) {
                cylinder_claim_srv_msg.request.cylinder_index = g_cylinder_pool[i];
                // call the cylinder claim service
                if (cylinder_pool_claim_client.call(cylinder_claim_srv_msg)) {
                    if (cylinder_claim_srv_msg.response.cylinder_claimed) {
                        // cylinder has been successfully claimed
                        cylinder_claimed = true;
                        cylinder_index = g_cylinder_pool[i];
                        break;
                    }
                }
            }
            if (!cylinder_claimed)
                ros::Duration(0.5).sleep();  // sleep for half a second
            ros::spinOnce();
        }
        ROS_INFO_STREAM("cylinder claimed by left scara: " << cylinder_index);
        // check upper boundary status of right scara
        bool left_in_action = false;
        double left_upper_boundary;
        if (g_right_in_action) {
            // right scara is in action, check boundary value for intersection
            // if left_boundary-right_boundary < 0.142, then there is intersection
            while ((g_cylinder_y[cylinder_index] - g_right_upper_boundary) < 0.142) {
                // there is boundary intersection
                ros::Duration(0.1).sleep();  // sleep for 0.1 second
            }
        }
        // no intersection anymore
        left_in_action = true;
        left_upper_boundary = g_cylinder_y[cylinder_index];
        // send boundary message to boundary topics maintainer
        left_boundary_change_srv.request.in_action_change = true;
        left_boundary_change_srv.request.in_action = left_in_action;
        left_boundary_change_srv.request.upper_boundary_change = true;
        left_boundary_change_srv.request.upper_boundary = left_upper_boundary;
        scara_left_upper_boundary_client.call(left_boundary_change_srv);
        if (left_boundary_change_srv.response.change_is_done)
            ROS_INFO_STREAM("left scara upper boundary updated in step 1");
        else
            ROS_INFO_STREAM("left scara upper boundary not update in step 1");
        
        // calculate target cylinder position
        std::vector<double> cylinder_position;
        cylinder_position.resize(2);
        cylinder_position[0] = g_cylinder_x[cylinder_index];
        cylinder_position[1] = g_cylinder_y[cylinder_index];




    }


}



