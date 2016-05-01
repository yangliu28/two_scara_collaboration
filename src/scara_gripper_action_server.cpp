// the action server for the gripper control
// except receiving command from client, it keeps the gripper in its last status

// ros communication:
    // 

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <two_scara_collaboration/scara_gripperAction.h>
#include <gazebo_msgs/ApplyJointEffort.h>




