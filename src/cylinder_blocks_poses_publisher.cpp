// this node will publish the topic "cylinder_blocks_poses"
// including all current cylinder blocks, pose is 3-D position

// ros communication:
    // subscribe to topic "/current_cylinder_blocks"
    // subscribe to topic "/gazebo/model_states"
    // publish the topic "/cylinder_blocks_poses"

#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <two_scara_collaboration/cylinder_blocks_poses.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "cylinder_blocks_poses_publisher");
    ros::NodeHandle nh;

    
}

