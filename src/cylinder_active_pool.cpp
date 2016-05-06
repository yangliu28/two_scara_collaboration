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
    // host a service for pool change "/cylinder_active_pool_claim"

#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <two_scara_collaboration/cylinder_blocks_poses.h>

// the upper and lower limit of reachable range along the direction of conveyor
const double RANGE_UPPER_LIMIT = sqrt(pow(1.8, 2) - pow(1.5, 2));
const double RANGE_LOWER_LIMIT = -sqrt(pow(1.8, 2) - pow(1.5, 2)) + 0.5;

// global variables
std::vector<int8_t> g_cylinder_active_pool;  // the POOL, elements are index of robots
// add cylinder to pool from this index when checking x position
// avoid repeated adding, some cylinders may be grasped by scara and back in range
int cylinder_add_start_index = 0;


// add and delete from the pool when in or out of the range
void cylinderPosesCallback(const two_scara_collaboration::cylinder_blocks_poses& cylinder_poses_msg) {
    // get the x coordinates
    std::vector<double> current_cylinder_x;
    int robot_quantity = cylinder_poses_msg.x.size();
    current_cylinder_x.resize(robot_quantity);
    current_cylinder_x = cylinder_poses_msg.x;

    // check for pool addition
    // add cylinders only after the cylinder_start_index
    for (int i=cylinder_start_index; i<robot_quantity; i++) {
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
    // this should not happen if the robots work well
    for (int i=0; i<g_cylinder_active_pool.size(); i++) {
        if (current_cylinder_x[g_cylinder_active_pool[i]] < RANGE_LOWER_LIMIT) {
            g_cylinder_active_pool.erase(i);
        }
    }
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
        = nh.advertise<std::vector<int8_t>>("/cylinder_active_pool", 1);

    // publish loop for the pool
    ros::Rate rate_timer(100);  // inside is the publish frequency
    while (ros::ok()) {
        // no need to be very fast, slower than "/cylinder_blocks_poses"
        cylinder_pool_publisher.publish(g_cylinder_active_pool);

        rate_timer.sleep();  // slow down publish rate
        ros::spinOnce();
    }

}

