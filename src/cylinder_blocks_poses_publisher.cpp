// this node will publish the topic "cylinder_blocks_poses"
// including all current cylinder blocks, pose is 3-D position

// ros communication:
    // subscribe to topic "/current_cylinder_blocks"
    // subscribe to topic "/gazebo/model_states"
    // publish the topic "/cylinder_blocks_poses"

#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Int8MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <two_scara_collaboration/cylinder_blocks_poses.h>

// global variables
int g_cylinder_quantity;
std::vector<int8_t> g_current_cylinder_blocks;
std::vector<double> g_x;
std::vector<double> g_y;
std::vector<double> g_z;
bool g_current_cylinder_callback_started = false;
bool g_cylinder_poses_updated = false;  // act as frequency control of publish loop

std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

void currentCylinderCallback(const std_msgs::Int8MultiArray& current_cylinder_blocks) {
    // this topic contains information of what cylinder blocks have been spawned
    if (!g_current_cylinder_callback_started) {
        // set first time started flag to true
        g_current_cylinder_callback_started = true;
        ROS_INFO("current cylinder callback has been invoked first time");
    }
    g_cylinder_quantity = current_cylinder_blocks.data.size();
    g_current_cylinder_blocks.resize(g_cylinder_quantity);
    g_current_cylinder_blocks = current_cylinder_blocks.data;
}

void modelStatesCallback(const gazebo_msgs::ModelStates& current_model_states) {
    // this callback update global values of cylinder positions
    if (g_current_cylinder_callback_started) {
        // only update when currentCylinderCallback has been invoked the first time
        // get cylinder blocks positions according to settings in g_current_cylinder_blocks
        std::vector<double> cylinder_x;
        std::vector<double> cylinder_y;
        std::vector<double> cylinder_z;
        cylinder_x.resize(g_cylinder_quantity);
        cylinder_y.resize(g_cylinder_quantity);
        cylinder_z.resize(g_cylinder_quantity);
        // find position of all current cylinders in topic message
        bool cylinder_poses_completed = true;
        for (int i=0; i<g_cylinder_quantity; i++) {
            // get index of ith cylinder
            std::string indexed_model_name;
            if (g_current_cylinder_blocks[i] == 0) {
                indexed_model_name = "red_cylinder_" + intToString(i);
            }
            else {
                indexed_model_name = "blue_cylinder_" + intToString(i);
            }
            int index = -1;
            int model_quantity = current_model_states.name.size();  // number of models measured
            for (int j=0; j<model_quantity; j++) {
                if (current_model_states.name[j] == indexed_model_name) {
                    index = j; break;
                }
            }
            if (index != -1) {
                // this model name has been successfully indexed
                cylinder_x[i] = current_model_states.pose[index].position.x;
                cylinder_y[i] = current_model_states.pose[index].position.y;
                cylinder_z[i] = current_model_states.pose[index].position.z;
            }
            else {
                // ROS_ERROR("fail to find model name in the model_states topic");
                // in the test run, there is chance that the last cylinder is not in the topic message
                // and g_cylinder_quantity (fron spawner node) is larger than the cylinder quantity here
                // because /gazebo/model_states are sampled at a high rate of about 1000Hz
                // so the position data should be aborted if fail to find the last cylinder
                cylinder_poses_completed = false;
            }
        }
        if (cylinder_poses_completed) {
            // only pass data to globals when they are completed
            g_x.resize(g_cylinder_quantity);
            g_y.resize(g_cylinder_quantity);
            g_z.resize(g_cylinder_quantity);
            g_x = cylinder_x;
            g_y = cylinder_y;
            g_z = cylinder_z;
            if (!g_cylinder_poses_updated) {
                // reset flag to true, after updating global value of g_x, g_y, g_z
                g_cylinder_poses_updated = true;
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cylinder_blocks_poses_publisher");
    ros::NodeHandle nh;

    // initialize subscribers for "/current_cylinder_blocks" and "/gazebo/model_states"
    ros::Subscriber current_cylinder_subscriber = nh.subscribe("/current_cylinder_blocks"
        , 1, currentCylinderCallback);
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
    // initialize publisher for "/cylinder_blocks_poses"
    ros::Publisher cylinder_poses_publisher
        = nh.advertise<two_scara_collaboration::cylinder_blocks_poses>("cylinder_blocks_poses", 1);
    two_scara_collaboration::cylinder_blocks_poses current_poses_msg;

    // publishing loop
    while (ros::ok()) {
        if (g_cylinder_poses_updated) {
            // only publish when cylinder positions are updated
            // no need to publish repeated data
            g_cylinder_poses_updated = false;  // set flag to false
            // there is tiny possibility that g_x is not in the length of g_cylinder_quantity
            int local_cylinder_quantity = g_x.size();  // so get length of g_x
            current_poses_msg.x.resize(local_cylinder_quantity);
            current_poses_msg.y.resize(local_cylinder_quantity);
            current_poses_msg.z.resize(local_cylinder_quantity);
            current_poses_msg.x = g_x;
            current_poses_msg.y = g_y;
            current_poses_msg.z = g_z;
            cylinder_poses_publisher.publish(current_poses_msg);
        }
        ros::spinOnce();
    }
    return 0;
}



