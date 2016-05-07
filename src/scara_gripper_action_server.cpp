// the action server for the gripper control
// except receiving command from client, it keeps the gripper in its last status

// ros communication:
    // host an action server "gripper_action" to receive command on the gripper
    // service client for gazebo service "/gazebo/apply_joint_effort"

// action server is not really necessary here
// because the server only receives commanded status of the gripper
// no feedback or result are need, a service server will also do fine

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <two_scara_collaboration/scara_gripperAction.h>
#include <gazebo_msgs/ApplyJointEffort.h>

// constants for joint effort and duration, values tuned
const double GRIPPER_UP_EFFORT = 0.001;
const double GRIPPER_DOWN_EFFORT = 0.0;
const double GRIPPER_GRASP_EFFORT_ABS = 0.002;  // absolute value
const double GRIPPER_RELEASE_EFFORT_ABS = 0.0001;  // absolute value
const ros::Duration EFFORT_DURATION(0, 250000000);  // 0.25s, longer than loop time of 0.1s
const double EFFORT_FREQUENCY = 1/0.1;  // this decide the reaction speed of action server

// global variables acted as state machine, to keep gripper status of last command
bool g_up_down_left = false;  // down is true, means stretch out gripper for task
bool g_up_down_right = false;
bool g_grasp_release_left = false;  // grasp is true
bool g_grasp_release_right = false;

// action server class definition
class GripperActionServer {
public:
    GripperActionServer();  // constructor
    ~GripperActionServer(void) {};  // destructor
    // action callback
    void executeCb(const
        actionlib::SimpleActionServer<two_scara_collaboration::scara_gripperAction>::GoalConstPtr& goal);

    // service client for "/gazebo/apply_joint_effort"
    // put service client in public so that it can be accessed from outside
    ros::ServiceClient joint_effort_client_;
    // service messages
    gazebo_msgs::ApplyJointEffort joint_effort_left0_srv_msg_;  // left0 indicate gripper joint
    gazebo_msgs::ApplyJointEffort joint_effort_left1_srv_msg_;  // left(i) indicate ith finger
    gazebo_msgs::ApplyJointEffort joint_effort_left2_srv_msg_;
    gazebo_msgs::ApplyJointEffort joint_effort_left3_srv_msg_;
    gazebo_msgs::ApplyJointEffort joint_effort_left4_srv_msg_;
    gazebo_msgs::ApplyJointEffort joint_effort_right0_srv_msg_;  // right0 indicate gripper joint
    gazebo_msgs::ApplyJointEffort joint_effort_right1_srv_msg_;  // right(i) indicate ith finger
    gazebo_msgs::ApplyJointEffort joint_effort_right2_srv_msg_;
    gazebo_msgs::ApplyJointEffort joint_effort_right3_srv_msg_;
    gazebo_msgs::ApplyJointEffort joint_effort_right4_srv_msg_;
private:
    ros::NodeHandle nh_;
    // create a "SimpleActionServer" call "as_"
    actionlib::SimpleActionServer<two_scara_collaboration::scara_gripperAction> as_;

    two_scara_collaboration::scara_gripperGoal goal_;  // goal message
    two_scara_collaboration::scara_gripperResult result_;  // result message
    two_scara_collaboration::scara_gripperFeedback feedback_;  // feedback message
};

// constructor
GripperActionServer::GripperActionServer():
as_(nh_, "gripper_action", boost::bind(&GripperActionServer::executeCb, this, _1), false)
{
    ROS_INFO("in constructor of GripperActionServer...");

    // initialize the apply joint effort service client
    joint_effort_client_
        = nh_.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");

    // check if service "/gazebo/apply_joint_effort" is ready
    bool service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/apply_joint_effort", true);
        ROS_INFO("waiting for apply_joint_effort service");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("apply_joint_effort service is ready");

    // preparation for some service message
    joint_effort_left0_srv_msg_.request.joint_name = "scara_robot_left::gripper_joint";  // left
    joint_effort_left1_srv_msg_.request.joint_name = "scara_robot_left::finger1_joint";
    joint_effort_left2_srv_msg_.request.joint_name = "scara_robot_left::finger2_joint";
    joint_effort_left3_srv_msg_.request.joint_name = "scara_robot_left::finger3_joint";
    joint_effort_left4_srv_msg_.request.joint_name = "scara_robot_left::finger4_joint";
    joint_effort_right0_srv_msg_.request.joint_name = "scara_robot_right::gripper_joint";  // right
    joint_effort_right1_srv_msg_.request.joint_name = "scara_robot_right::finger1_joint";
    joint_effort_right2_srv_msg_.request.joint_name = "scara_robot_right::finger2_joint";
    joint_effort_right3_srv_msg_.request.joint_name = "scara_robot_right::finger3_joint";
    joint_effort_right4_srv_msg_.request.joint_name = "scara_robot_right::finger4_joint";
    ros::Time time_temp(0, 0);
    joint_effort_left0_srv_msg_.request.start_time = time_temp;  // set the time to immediately
    joint_effort_left1_srv_msg_.request.start_time = time_temp;
    joint_effort_left2_srv_msg_.request.start_time = time_temp;
    joint_effort_left3_srv_msg_.request.start_time = time_temp;
    joint_effort_left4_srv_msg_.request.start_time = time_temp;
    joint_effort_right0_srv_msg_.request.start_time = time_temp;
    joint_effort_right1_srv_msg_.request.start_time = time_temp;
    joint_effort_right2_srv_msg_.request.start_time = time_temp;
    joint_effort_right3_srv_msg_.request.start_time = time_temp;
    joint_effort_right4_srv_msg_.request.start_time = time_temp;
    joint_effort_left0_srv_msg_.request.duration = EFFORT_DURATION;  // set the duration
    joint_effort_left1_srv_msg_.request.duration = EFFORT_DURATION;
    joint_effort_left2_srv_msg_.request.duration = EFFORT_DURATION;
    joint_effort_left3_srv_msg_.request.duration = EFFORT_DURATION;
    joint_effort_left4_srv_msg_.request.duration = EFFORT_DURATION;
    joint_effort_right0_srv_msg_.request.duration = EFFORT_DURATION;
    joint_effort_right1_srv_msg_.request.duration = EFFORT_DURATION;
    joint_effort_right2_srv_msg_.request.duration = EFFORT_DURATION;
    joint_effort_right3_srv_msg_.request.duration = EFFORT_DURATION;
    joint_effort_right4_srv_msg_.request.duration = EFFORT_DURATION;

    as_.start();  // start the action server
}

// callback of the action server
void GripperActionServer::executeCb(const
    actionlib::SimpleActionServer<two_scara_collaboration::scara_gripperAction>::GoalConstPtr& goal)
{
    ROS_INFO("in gripper_action callback...");
    ROS_INFO_STREAM("the goal message received:");
    ROS_INFO_STREAM("left gripper setting: " << g_up_down_left <<
        ", " << g_grasp_release_left);
    ROS_INFO_STREAM("right gripper setting: " << g_up_down_right <<
        ", " << g_grasp_release_right);

    // up down position of left gripper
    if (goal->up_down_left == -1) {
        g_up_down_left = false;
    }
    else if (goal->up_down_left == 1) {
        g_up_down_left = true;
    }  // if 0 then change nothing
    // up down position of right gripper
    if (goal->up_down_right == -1) {
        g_up_down_right = false;
    }
    else if (goal->up_down_right == 1) {
        g_up_down_right = true;
    }
    // grasp release of left gripper
    if (goal->grasp_release_left == -1) {
        g_grasp_release_left = false;
    }
    else if (goal->grasp_release_left == 1) {
        g_grasp_release_left = true;
    }
    // grasp release of right gripper
    if (goal->grasp_release_right == -1) {
        g_grasp_release_right = false;
    }
    else if (goal->grasp_release_right == 1) {
        g_grasp_release_right = true;
    }

    result_.result = 1;  // the result value doesn't matter
    as_.setSucceeded(result_);  // tell the client this action is finished
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "scara_gripper_action_server");

    GripperActionServer as_object;  // instance of the class

    // apply all joint effort loop
    ros::Rate loop_rate(EFFORT_FREQUENCY);  // loop frequency control
    ros::Time time_last, time_now;
    time_last  = ros::Time::now();
    ros::Duration loop_duration;
    while (ros::ok()) {
        // loop time measurement
        time_now = ros::Time::now();
        loop_duration = time_now - time_last;
        time_last = time_now;
        // ROS_INFO_STREAM("loop duration: " << loop_duration.toSec());

        // prepare joint effort message
        // left gripper
        if (g_up_down_left == false) {  // gripper goes up
            as_object.joint_effort_left0_srv_msg_.request.effort = GRIPPER_UP_EFFORT;
        }
        else {  // gripper goes down
            as_object.joint_effort_left0_srv_msg_.request.effort = GRIPPER_DOWN_EFFORT;
        }
        if (g_grasp_release_left == false) {  // left gripper release
            as_object.joint_effort_left1_srv_msg_.request.effort = -GRIPPER_RELEASE_EFFORT_ABS;
            as_object.joint_effort_left2_srv_msg_.request.effort = GRIPPER_RELEASE_EFFORT_ABS;
            as_object.joint_effort_left3_srv_msg_.request.effort = GRIPPER_RELEASE_EFFORT_ABS;
            as_object.joint_effort_left4_srv_msg_.request.effort = -GRIPPER_RELEASE_EFFORT_ABS;
        }
        else {  // left gripper grasp
            as_object.joint_effort_left1_srv_msg_.request.effort = GRIPPER_GRASP_EFFORT_ABS;
            as_object.joint_effort_left2_srv_msg_.request.effort = -GRIPPER_GRASP_EFFORT_ABS;
            as_object.joint_effort_left3_srv_msg_.request.effort = -GRIPPER_GRASP_EFFORT_ABS;
            as_object.joint_effort_left4_srv_msg_.request.effort = GRIPPER_GRASP_EFFORT_ABS;
        }
        // right gripper
        if (g_up_down_right == false) {  // gripper goes up
            as_object.joint_effort_right0_srv_msg_.request.effort = GRIPPER_UP_EFFORT;
        }
        else {  // gripper goes down
            as_object.joint_effort_right0_srv_msg_.request.effort = GRIPPER_DOWN_EFFORT;
        }
        if (g_grasp_release_right == false) {  // right gripper release
            as_object.joint_effort_right1_srv_msg_.request.effort = GRIPPER_RELEASE_EFFORT_ABS;
            as_object.joint_effort_right2_srv_msg_.request.effort = -GRIPPER_RELEASE_EFFORT_ABS;
            as_object.joint_effort_right3_srv_msg_.request.effort = -GRIPPER_RELEASE_EFFORT_ABS;
            as_object.joint_effort_right4_srv_msg_.request.effort = GRIPPER_RELEASE_EFFORT_ABS;
        }
        else {  // right gripper grasp
            as_object.joint_effort_right1_srv_msg_.request.effort = -GRIPPER_GRASP_EFFORT_ABS;
            as_object.joint_effort_right2_srv_msg_.request.effort = GRIPPER_GRASP_EFFORT_ABS;
            as_object.joint_effort_right3_srv_msg_.request.effort = GRIPPER_GRASP_EFFORT_ABS;
            as_object.joint_effort_right4_srv_msg_.request.effort = -GRIPPER_GRASP_EFFORT_ABS;
        }

        // send service message to gazebo
        as_object.joint_effort_client_.call(as_object.joint_effort_left0_srv_msg_);
        as_object.joint_effort_client_.call(as_object.joint_effort_left1_srv_msg_);
        as_object.joint_effort_client_.call(as_object.joint_effort_left2_srv_msg_);
        as_object.joint_effort_client_.call(as_object.joint_effort_left3_srv_msg_);
        as_object.joint_effort_client_.call(as_object.joint_effort_left4_srv_msg_);
        as_object.joint_effort_client_.call(as_object.joint_effort_right0_srv_msg_);
        as_object.joint_effort_client_.call(as_object.joint_effort_right1_srv_msg_);
        as_object.joint_effort_client_.call(as_object.joint_effort_right2_srv_msg_);
        as_object.joint_effort_client_.call(as_object.joint_effort_right3_srv_msg_);
        as_object.joint_effort_client_.call(as_object.joint_effort_right4_srv_msg_);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}


