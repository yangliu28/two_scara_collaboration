// copy from scara_right_motion_planner

// test program for scara motion control
// control the scara robot by publishing joint angles on corresponding topic
// one of the two independent motion planner
// all two scara robots are positive configured

// ros communication:
    // subscribe to topic "/current_cylinder_blocks"
    // subscribe to topic "/cylinder_blocks_poses"
    // subscribe to topic "/scara_robot_right_rotation1_pos"
    // subscribe to topic "/scara_robot_right_rotation2_pos"
    // publish to topic "/scara_robot_right_rotation1_pos_cmd"
    // publish to topic "/scara_robot_right_rotation2_pos_cmd"
    // subscribe to topic "/cylinder_active_pool"
    // service client to service "/cylinder_pool_claim"
    // subscribe to topic "/scara_left_upper_boundary"
    // service client to service "/scara_right_upper_boundary_change"
    // action client for gripper control, "gripper_action"

// motion steps decomposed:
    // 1.stand by position -> target cylinder position
    // 2.target cylinder hovering (for cylinder grasping operation)
    // 3.target cylinder position -> drop position
    // 4.drop position -> stand by position
// only in motion step 1, the operation may be delayed when the other robot is in operation range

// upper boundary control method:
    // each robot has a state machine contains a boolean and a number
    // the boolean indicate if the robot is at upper area of the conveyor
    // the number indicate the upper boundary the robot goes
    // each robot check the other robot's state and make change to its own


#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64.h>
#include <two_scara_collaboration/cylinder_blocks_poses.h>
#include <two_scara_collaboration/pool_claim_msg.h>
#include <std_msgs/Int8MultiArray.h>
#include <two_scara_collaboration/scara_upper_boundary.h>
#include <two_scara_collaboration/upper_boundary_change.h>
#include <actionlib/client/simple_action_client.h>
#include <two_scara_collaboration/scara_gripperAction.h>

const double CYLINDER_SPEED = 0.12;  // the measured absolute speed of the cylinder
const double STEP1_DURANCE = 2.0;  // time allocation for step 1
const double STEP2_DURANCE = 4.0;  // time allocation for step 2
const double STEP3_DURANCE = 1.0;  // time allocation for step 3
const double STEP4_DURANCE = 1.0;  // time allocation for step 4
const double MOTION_SAMPLE_TIME = 0.01;

const double RED_DROP_JOINT1 = 0.3;  // drop position for red cylinder
const double RED_DROP_JOINT2 = 2.0;
const double BLUE_DROP_JOINT1 = -1.9;  // drop position for blue cylinder
const double BLUE_DROP_JOINT2 = 1.8;
const double STAND_BY_JOINT1 = -0.78;  // stand by scara joints
const double STAND_BY_JOINT2 = 2.1;

const double MAGIC_DISP = 0.03;  // real cylinder position compensation

// global variables
std::vector<int8_t> g_current_cylinders;  // 0 or 1 represent red or blue
bool g_current_cylinders_initialized = false;
std::vector<double> g_cylinder_x;  // only x y coordinates matter
std::vector<double> g_cylinder_y;
double g_scara_right_r1;  // the two joints of scara
double g_scara_right_r2;
std::vector<int8_t> g_cylinder_pool;  // the active pool
bool g_left_in_action;  // upper boundary status of left scara
double g_left_upper_boundary;


// kinematics for the right scara robot
std::vector<double> scara_right_kinematics(std::vector<double> joints) {
    // x_generic and y_generic for the generic coordinates used in scara kinematics
    double x_generic = cos(joints[0]) + 0.8*cos(joints[0] + joints[1]);
    double y_generic = sin(joints[0]) + 0.8*sin(joints[0] + joints[1]);
    // convert to coordinates in the view of right scara
    std::vector<double> output_position;
    output_position.resize(2);
    output_position[0] = -y_generic;  // x coordinate
    output_position[1] = x_generic - 1.5;  // y coordinate
    return output_position;
}

// inverse kinematics for the right scara robot
std::vector<double> scara_right_inverse_kinematics(std::vector<double> position) {
    // the returned robot configuration is always positive
    // convert position to the generic coordinates in scara kinematics
    double x_generic = position[1] + 1.5;
    double y_generic = -position[0];
    std::vector<double> output_joints;
    output_joints.resize(2);
    double end_quad_dist = pow(x_generic, 2) + pow(y_generic, 2);
    output_joints[0] = atan(y_generic/x_generic) - acos((0.36 + end_quad_dist)/(2*sqrt(end_quad_dist)));
    output_joints[1] = M_PI - acos((1.64 - end_quad_dist)/1.6);
    // if (abs(output_joints[1]) < 0.2) {
    //     ROS_INFO_STREAM("IK points: " << x_generic << ", " << y_generic);
    //     ROS_INFO_STREAM("IK joints: " << output_joints[0] << ", " << output_joints[1]);
    // }
    // add robust to this inverse kinematics
    if ((output_joints[0] != output_joints[0]) || (output_joints[1] != output_joints[1])) {
        // there nan in the output, i.e., value inside cos() is out of [-1,1]
        output_joints[0] = atan(y_generic/x_generic);
        output_joints[1] = 0;
    }
    return output_joints;
}

void currentCylinderCallback(const std_msgs::Int8MultiArray& current_cylinder_msg) {
    // update current cylinder blocks
    if (!g_current_cylinders_initialized)
        g_current_cylinders_initialized = true;  // set flag to true
    g_current_cylinders.resize(current_cylinder_msg.data.size());
    g_current_cylinders = current_cylinder_msg.data;
}

void cylinderPosesCallback(const two_scara_collaboration::cylinder_blocks_poses& cylinder_poses_msg) {
    // update poses message in global variables
    int cylinder_quantity = cylinder_poses_msg.x.size();
    g_cylinder_x.resize(cylinder_quantity);
    g_cylinder_y.resize(cylinder_quantity);
    g_cylinder_x = cylinder_poses_msg.x;
    g_cylinder_y = cylinder_poses_msg.y;
}

void scararightR1PosCallback(const std_msgs::Float64& rotation1_msg) {
    // update first joint angle of right scara robot
    g_scara_right_r1 = rotation1_msg.data;
}

void scararightR2PosCallback(const std_msgs::Float64& rotation2_msg) {
    // update second joint angle of right scara robot
    g_scara_right_r2 = rotation2_msg.data;
}

void cylinderPoolCallback(const std_msgs::Int8MultiArray& cylinder_pool_msg) {
    // update the cylinder active pool
    int pool_size = cylinder_pool_msg.data.size();
    g_cylinder_pool.resize(pool_size);
    g_cylinder_pool = cylinder_pool_msg.data;
}

void scaraleftUpperBoundaryCallback(const
    two_scara_collaboration::scara_upper_boundary& upper_boundary_msg) {
    // update on upper boundary of left scara
    g_left_in_action = upper_boundary_msg.in_action;
    g_left_upper_boundary = upper_boundary_msg.upper_boundary;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "scara_right_motion_planner");
    ros::NodeHandle nh;

    ROS_ERROR("program stamp 1");

    // initialize subscriber to "/current_cylinder_blocks"
    ros::Subscriber current_cylinder_subscriber = nh.subscribe("/current_cylinder_blocks"
        , 1, currentCylinderCallback);
    // initialize subscriber to "/cylinder_blocks_poses"
    ros::Subscriber cylinder_poses_subscriber = nh.subscribe("/cylinder_blocks_poses"
        , 1, cylinderPosesCallback);
    // initialize subscriber to "/scara_robot_right_rotation1_pos"
    ros::Subscriber scara_right_r1_pos_subscriber = nh.subscribe("/scara_robot_right_rotation1_pos"
        , 1, scararightR1PosCallback);
    // initialize subscriber to "/scara_robot_right_rotation2_pos"
    ros::Subscriber scara_right_r2_pos_subscriber = nh.subscribe("/scara_robot_right_rotation2_pos"
        , 1, scararightR2PosCallback);
    // publish to topic "/scara_robot_right_rotation1_pos_cmd"
    ros::Publisher scara_right_r1_cmd_publisher
        = nh.advertise<std_msgs::Float64>("/scara_robot_right_rotation1_pos_cmd", 1);
    std_msgs::Float64 scara_right_r1_cmd_msg;
    // publish to topic "/scara_robot_right_rotation2_pos_cmd"
    ros::Publisher scara_right_r2_cmd_publisher
        = nh.advertise<std_msgs::Float64>("/scara_robot_right_rotation2_pos_cmd", 1);
    std_msgs::Float64 scara_right_r2_cmd_msg;
    // subscribe to topic "/cylinder_active_pool"
    ros::Subscriber cylinder_pool_subscriber = nh.subscribe("/cylinder_active_pool"
        , 1, cylinderPoolCallback);
    // service client to service "/cylinder_pool_claim"
    ros::ServiceClient cylinder_pool_claim_client
        = nh.serviceClient<two_scara_collaboration::pool_claim_msg>("/cylinder_pool_claim");
    two_scara_collaboration::pool_claim_msg cylinder_claim_srv_msg;
    // subscribe to topic "/scara_left_upper_boundary"
    ros::Subscriber scara_left_upper_boundary_subscriber
        = nh.subscribe("/scara_left_upper_boundary", 1, scaraleftUpperBoundaryCallback);
    // service client to service "/scara_right_upper_boundary_change"
    ros::ServiceClient scara_right_upper_boundary_client
        = nh.serviceClient<two_scara_collaboration::upper_boundary_change>("/scara_right_upper_boundary_change");
    two_scara_collaboration::upper_boundary_change right_boundary_change_srv;    
    // action client for gripper control, "gripper_action"
    actionlib::SimpleActionClient<two_scara_collaboration::scara_gripperAction>
        scara_gripper_action_client("gripper_action", true);
    two_scara_collaboration::scara_gripperGoal scara_gripper_goal;
    scara_gripper_goal.up_down_left = 0;
    scara_gripper_goal.grasp_release_left = 0;

    ROS_ERROR("program stamp 2");

    while (!g_current_cylinders_initialized) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_INFO("current_cylinder_blocks initialized");


    // the motion planner loop
    int count;  // the motion count
    int cylinder_index;  // the index of claimed cylinder
    bool right_in_action = false;  // right scara boundary status
    double right_upper_boundary;
    std::vector<double> cylinder_position;  // the dynamic cylinder position
    cylinder_position.resize(2);
    std::vector<double> cylinder_joints;  // the dynamic scara joints of cylinder
    cylinder_joints.resize(2);
    std::vector<double> scara_joints;  // the dynamic scara joints position
    scara_joints.resize(2);
    std::vector<double> scara_position;  // the dynamic position of scara
    scara_position.resize(2);
    std::vector<double> scara_position_new;  // the computed new position of scara
    scara_position_new.resize(2);
    std::vector<double> scara_joints_new;  // the computed new joints of scara
    scara_joints_new.resize(2);
    std::vector<double> cylinder_drop_joints;  // cylinder drop position in scara joints
    cylinder_drop_joints.resize(2);
    std::vector<double> cylinder_drop_position;  // cylinder drop position
    cylinder_drop_position.resize(2);
    std::vector<double> stand_by_joints;  // stand by scara joints
    stand_by_joints.resize(2);
    std::vector<double> stand_by_position;  // stand by scara position
    stand_by_position.resize(2);
    while (ros::ok()) {
        // motion has been divided into four parts according to the purposes

        ROS_ERROR("program motion 1 start");

        // 1.stand by position -> target cylinder position
        // claim cylinders from cylinder active pool
        bool cylinder_claimed = false;
        while (!cylinder_claimed) {
            // cylinder not claimed
            for (int i=0; i<g_cylinder_pool.size(); i++) {
                if (g_cylinder_y[g_cylinder_pool[i]] > 0.25)  // can't reach
                    continue;
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
        ROS_INFO_STREAM("cylinder claimed by right scara: " << cylinder_index);
        // check upper boundary status of left scara
        if (g_left_in_action) {
            // left scara is in action, check boundary value for intersection
            // if left_boundary-right_boundary < 0.142, then there is intersection
            while ((g_left_upper_boundary - g_cylinder_y[cylinder_index]) < 0.142) {
                // there is boundary intersection
                ros::Duration(0.1).sleep();  // sleep for 0.1 second
            }
        }
        // no intersection anymore
        right_in_action = true;
        right_upper_boundary = g_cylinder_y[cylinder_index];
        // send boundary message to boundary topics maintainer
        right_boundary_change_srv.request.in_action_change = true;
        right_boundary_change_srv.request.in_action = right_in_action;
        right_boundary_change_srv.request.upper_boundary_change = true;
        right_boundary_change_srv.request.upper_boundary = right_upper_boundary;
        scara_right_upper_boundary_client.call(right_boundary_change_srv);
        if (right_boundary_change_srv.response.change_is_done)
            ROS_INFO_STREAM("right scara upper boundary updated in step 1");
        else
            ROS_INFO_STREAM("right scara upper boundary not update in step 1");
        // step 1 motion control
        count = STEP1_DURANCE / MOTION_SAMPLE_TIME;
        for (int i=0; i<count; i++) {
            // in each step, always heading to the updated cylinder position
            cylinder_position[0] = g_cylinder_x[cylinder_index] - MAGIC_DISP;
            cylinder_position[1] = g_cylinder_y[cylinder_index];
            cylinder_joints = scara_right_inverse_kinematics(cylinder_position);
            // prepare joints command and publish them
            scara_right_r1_cmd_msg.data = cylinder_joints[0];
            scara_right_r2_cmd_msg.data = cylinder_joints[1];
            scara_right_r1_cmd_publisher.publish(scara_right_r1_cmd_msg);
            scara_right_r2_cmd_publisher.publish(scara_right_r2_cmd_msg);
            // delay and update
            ros::Duration(MOTION_SAMPLE_TIME).sleep();
            ros::spinOnce();
        }
        // it can be assumed now that right scara is hovering over target cylinder

        ROS_ERROR("program motion 2 start");

        // 2.target cylinder hovering (for cylinder grasping operation)
        // define durance for this step is 2 second, the grasping behaviros hasspens at
            // 0.5 second: lower the gripper
            // 1.0 second: grasp the cylinder
            // 1.5 second: lift the gripper
        // for cylinder hovering, always command target position at the cylinder
        count = STEP2_DURANCE / MOTION_SAMPLE_TIME; // reuse count
        for (int i=0; i<count; i++) {
            // get current cylinder position, scara position not needed
            scara_position_new[0] = g_cylinder_x[cylinder_index] - MAGIC_DISP;
            scara_position_new[1] = g_cylinder_y[cylinder_index];
            scara_joints_new = scara_right_inverse_kinematics(scara_position_new);
            // prepare joints command and publish them
            scara_right_r1_cmd_msg.data = scara_joints_new[0];
            scara_right_r2_cmd_msg.data = scara_joints_new[1];
            scara_right_r1_cmd_publisher.publish(scara_right_r1_cmd_msg);
            scara_right_r2_cmd_publisher.publish(scara_right_r2_cmd_msg);
            // gripper actions
            if (i == 100) {
                // at 1.0 second, let gripper go down
                scara_gripper_goal.up_down_right = 1;
                scara_gripper_goal.grasp_release_right = 0;
                scara_gripper_action_client.sendGoal(scara_gripper_goal);
            }
            else if (i == 200) {
                // at 2.0 second, let the gripper grasp
                scara_gripper_goal.up_down_right = 0;
                scara_gripper_goal.grasp_release_right = 1;
                scara_gripper_action_client.sendGoal(scara_gripper_goal);
            }
            else if (i == 300) {
                // at 3.0 second, let the gripper lift up
                scara_gripper_goal.up_down_right = -1;
                scara_gripper_goal.grasp_release_right = 0;
                scara_gripper_action_client.sendGoal(scara_gripper_goal);
            }
            // delay and update
            ros::Duration(MOTION_SAMPLE_TIME).sleep();
            ros::spinOnce();
        }
        // it can be assumed that right gripper has got the cylinder

        ROS_ERROR("program motion 3 start");

        // 3.target cylinder position -> drop position
        // update the upper boundary dynamically, set in_action to false at the end
        // calculate target position, i.e., drop position
        if (g_current_cylinders[cylinder_index] == 0) {
            // goes to red cylinder drop position
            cylinder_drop_joints[0] = RED_DROP_JOINT1;
            cylinder_drop_joints[1] = RED_DROP_JOINT2;
            cylinder_drop_position = scara_right_kinematics(cylinder_drop_joints);
        }
        else {
            // goes to blue cylinder drop position
            cylinder_drop_joints[0] = BLUE_DROP_JOINT1;
            cylinder_drop_joints[1] = BLUE_DROP_JOINT2;
            cylinder_drop_position = scara_right_kinematics(cylinder_drop_joints);
        }
        ROS_INFO_STREAM("drop position: " << cylinder_drop_position[0] << ", " << cylinder_drop_position[1]);
        ROS_INFO_STREAM("drop joints: " << cylinder_drop_joints[0] << ", " << cylinder_drop_joints[1]);
        count = STEP3_DURANCE / MOTION_SAMPLE_TIME;
        for (int i=0; i<count; i++) {
            // get current scara joints and position
            scara_joints[0] = g_scara_right_r1;
            scara_joints[1] = g_scara_right_r2;
            scara_position = scara_right_kinematics(scara_joints);
            // compute new scara position
            // scara_position_new[0] = ((count-i-1)*scara_position[0] + cylinder_drop_position[0])/(count-i);
            // scara_position_new[1] = ((count-i-1)*scara_position[1] + cylinder_drop_position[1])/(count-i);
            // scara_joints_new = scara_right_inverse_kinematics(scara_position_new);
            scara_position_new[0] = cylinder_drop_position[0];
            scara_position_new[1] = cylinder_drop_position[1];
            scara_joints_new = scara_right_inverse_kinematics(scara_position_new);
            // prepare joints command and publish them
            scara_right_r1_cmd_msg.data = scara_joints_new[0];
            scara_right_r2_cmd_msg.data = scara_joints_new[1];
            scara_right_r1_cmd_publisher.publish(scara_right_r1_cmd_msg);
            scara_right_r2_cmd_publisher.publish(scara_right_r2_cmd_msg);
            // update upper boundary information
            right_boundary_change_srv.request.in_action_change = false;
            right_boundary_change_srv.request.upper_boundary_change = true;
                // the y position as the upper boundary
            right_boundary_change_srv.request.upper_boundary = scara_position[1];
            scara_right_upper_boundary_client.call(right_boundary_change_srv);  // call the service
            // delay and update
            ros::Duration(MOTION_SAMPLE_TIME).sleep();
            ros::spinOnce();
        }
        // now it can be assumed that gripper has reached drop location
        // set again the upper boundary inforamtion
        right_boundary_change_srv.request.in_action_change = true;
        right_boundary_change_srv.request.in_action = false;
        right_boundary_change_srv.request.upper_boundary_change = false;
        scara_right_upper_boundary_client.call(right_boundary_change_srv);  // call the service
        // drop the cylinder operation
        scara_gripper_goal.up_down_right = 0;
        scara_gripper_goal.grasp_release_right = -1;
        scara_gripper_action_client.sendGoal(scara_gripper_goal);
        ros::Duration(0.5).sleep();  // delay for operation to finish

        ROS_ERROR("program motion 4 start");

        // 4.drop position -> stand by position
        stand_by_joints[0] = STAND_BY_JOINT1;
        stand_by_joints[1] = STAND_BY_JOINT2;
        stand_by_position = scara_right_kinematics(stand_by_joints);
        ROS_INFO_STREAM("stand by position: " << stand_by_position[0] << ", " << stand_by_position[1]);
        ROS_INFO_STREAM("stand by joints: " << stand_by_joints[0] << ", " << stand_by_joints[1]);
        count = STEP4_DURANCE / MOTION_SAMPLE_TIME;
        for (int i=0; i<count; i++) {
            // get current scara joints and position
            scara_joints[0] = g_scara_right_r1;
            scara_joints[1] = g_scara_right_r2;
            scara_position = scara_right_kinematics(scara_joints);
            // prepare joints command and publish them
            scara_right_r1_cmd_msg.data = stand_by_joints[0];
            scara_right_r2_cmd_msg.data = stand_by_joints[1];
            scara_right_r1_cmd_publisher.publish(scara_right_r1_cmd_msg);
            scara_right_r2_cmd_publisher.publish(scara_right_r2_cmd_msg);
            // delay and update
            ros::Duration(MOTION_SAMPLE_TIME).sleep();
            ros::spinOnce();
        }

    }

    return 0;

}



