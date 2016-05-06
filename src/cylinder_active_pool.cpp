// this node maintain a pool of reachable blocks
// reachable means in the range of the the scara robots
// and could be fetched before it leaves robot's range
// this cylinder active pool has follow read/write property:
    // published for public read access
    // write by sending a service request

// add into the pool only when it goes into the reachable range
// delete from the pool
    // when it goes out the range
    // when it is claimed by one of the robot motion planner

// ros communication:
    // subscribe to topic "/cylinder_blocks_poses"
    // publish the topic "/cylinder_active_pool"
    // host a service for pool change "/cylinder_pool_claim"

#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <two_scara_collaboration/cylinder_blocks_poses.h>
#include <two_scara_collaboration/pool_claim_msg.h>
#include <std_msgs/Int8MultiArray.h>

// the upper and lower limit of reachable range along the direction of conveyor
const double RANGE_UPPER_LIMIT = sqrt(pow(1.8, 2) - pow(1.5, 2));
const double RANGE_LOWER_LIMIT = -sqrt(pow(1.8, 2) - pow(1.5, 2)) + 0.5;

// global variables
std::vector<int8_t> g_cylinder_active_pool;  // the POOL, elements are index of cylinders
// add cylinder to pool from this index when checking x position
// avoid repeated adding, some cylinders may be grasped by scara and back in range
int cylinder_add_start_index = 0;


// add and delete from the pool when in or out of the range
void cylinderPosesCallback(const two_scara_collaboration::cylinder_blocks_poses& cylinder_poses_msg) {
    // get the x coordinates
    std::vector<double> current_cylinder_x;
    int cylinder_quantity = cylinder_poses_msg.x.size();
    current_cylinder_x.resize(cylinder_quantity);
    current_cylinder_x = cylinder_poses_msg.x;

    // check for pool addition
    // add cylinders only after the cylinder_start_index
    for (int i=cylinder_add_start_index; i<cylinder_quantity; i++) {
        if (current_cylinder_x[i] > RANGE_LOWER_LIMIT
            && current_cylinder_x[i] < RANGE_UPPER_LIMIT) {
            bool in_pool = false;  // presume not in the pool
            for (int j=0; j<g_cylinder_active_pool.size(); j++) {
                if (g_cylinder_active_pool[j] == i) {
                    in_pool = true;
                    break;  // jump out once found in pool
                }
            }
            if (!in_pool) {
                // push into the pool
                g_cylinder_active_pool.push_back(i);
                cylinder_add_start_index = i + 1;  // update index to the next cylinder
            }
        }
    }

    // check for pool removal
    // this should not happen if the robots are working claim cylinder from pool
    for (int i=0; i<g_cylinder_active_pool.end() - g_cylinder_active_pool.begin(); i++) {
        // need use end()-begin(), because the vector length may change
        if (current_cylinder_x[g_cylinder_active_pool[i]] < RANGE_LOWER_LIMIT) {
            // remove ith cylinder from pool
            g_cylinder_active_pool.erase(g_cylinder_active_pool.begin() + i);
        }
    }
}

bool cylinderPoolCallback(two_scara_collaboration::pool_claim_msgRequest& request
    , two_scara_collaboration::pool_claim_msgResponse& response) {
    // service callback for robot to claim cylinder from the active pool
    ROS_INFO_STREAM("");
    ROS_WARN("in the /cylinder_pool_claim service");  // just to highlight the result
    int requested_cylinder_index = request.cylinder_index;
    response.cylinder_claimed = false;  // preset to not found
    for (int i=0; i<g_cylinder_active_pool.size(); i++) {
        if (g_cylinder_active_pool[i] == requested_cylinder_index) {
            // found in the pool, remove the cylinder
            g_cylinder_active_pool.erase(g_cylinder_active_pool.begin() + i);
            response.cylinder_claimed = true;
            break;
        }
    }
    // print out result of this call
    if (response.cylinder_claimed == false)
        ROS_INFO_STREAM("cylinder " << requested_cylinder_index << " fail to be claimed");
    else
        ROS_INFO_STREAM("cylinder " << requested_cylinder_index << " claimed");
    ROS_INFO_STREAM("");  // another blank line to space out above message
    return response.cylinder_claimed;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "cylinder_active_pool");
    ros::NodeHandle nh;

    // initialize the active pool
    g_cylinder_active_pool.resize(0);
    g_cylinder_active_pool.clear();

    // initialize subscriber to "/cylinder_blocks_poses"
    ros::Subscriber cylinder_poses_subscriber = nh.subscribe("/cylinder_blocks_poses"
        , 1, cylinderPosesCallback);
    // initialize publisher to "/cylinder_active_pool"
    ros::Publisher cylinder_pool_publisher
        = nh.advertise<std_msgs::Int8MultiArray>("/cylinder_active_pool", 1);
    std_msgs::Int8MultiArray cylinder_active_pool_msg;
    // initialize service server of "/cylinder_pool_claim"
    ros::ServiceServer cylinder_pool_server
        = nh.advertiseService("/cylinder_pool_claim", cylinderPoolCallback);

    // publish loop for the pool
    // inside is publish frequency, should be lower than "/cylinder_blocks_poses"
    // "/cylinder_blocks_poses" is measured at about 200hz
    ros::Rate rate_timer(100);
    while (ros::ok()) {
        cylinder_active_pool_msg.data.clear();
        cylinder_active_pool_msg.data = g_cylinder_active_pool;
        cylinder_pool_publisher.publish(cylinder_active_pool_msg);

        rate_timer.sleep();  // slow down publish rate
        ros::spinOnce();
    }

}

