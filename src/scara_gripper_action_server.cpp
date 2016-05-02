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

// constants for joint effort and duration
const double gripper_up_effort = 0.1;
const double gripper_down_effort = 0.1;
const double gripper_grasp_effort = 0.1;
const double gripper_release_effort = 0.1;

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
    as_.setSucceeced(result_);  // tell the client this action is finished
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "scara_gripper_action_server");

    GripperActionServer action_server_object;  // instance of the class

    // service client for "/gazebo/apply_joint_effort"
    ros::ServiceClient joint_effort_client
        = nh_.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    gazebo_msgs::ApplyJointEffort joint_effort_srv_msg;  // service message

    // check if service "/gazebo/apply_joint_effort" is ready
    bool service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/apply_joint_effort", true);
        ROS_INFO("waiting for apply_joint_effort service");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("apply_joint_effort service is ready");

    // in the loop, keep sending joint effort according to global gripper setting
    while (ros::ok()) {
        // prepare joint effort message and send to gazebo
        joint_effort_srv_msg.joint_name = "scara_robot_left::gripper_joint";



        ros::spinOnce();
    }

    return 0;
}


