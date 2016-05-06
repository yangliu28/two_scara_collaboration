// this node maintain a pool of reachable blocks
// reachable means in the range of the the scara robots
// and could be fetched before it leaves robot's range
// this cylinder active pool has follow read/write property:
    // published for public read access
    // write by sending a service request

// ros communication:
    // subscribe to topic "/cylinder_blocks_poses"
    // publish the topic "/cylinder_active_pool"
    // host a service for pool change "/cylinder_active_pool_claim"

#include <ros/ros.h>




