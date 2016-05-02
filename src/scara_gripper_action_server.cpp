// the action server for the gripper control
// except receiving command from client, it keeps the gripper in its last status

// ros communication:
    // host an action server "gripper_action" to receive command on the gripper
    // 


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <two_scara_collaboration/scara_gripperAction.h>
#include <gazebo_msgs/ApplyJointEffort.h>

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

}



