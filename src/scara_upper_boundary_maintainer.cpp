// a node that publish on following two topics
    // "/scara_left_upper_boundary"
    // "/scara_left_upper_boundary"
// these two topics describe the upper boundary in their planned motion
// used as state machines for the purpose of collision avoidance

// ros communication:
    // keep publishing on upper boundary topics
    // host two service servers for published data change
        // "/scara_left_upper_boundary_change"
        // "/scara_right_upper_boundary_change"


#include <ros/ros.h>
#include <two_scara_collaboration/scara_upper_boundary.h>
#include <two_scara_collaboration/upper_boundary_change.h>

// global data
bool g_

bool left_change_server(two_scara_collaboration::upper_boundary_changeRequest& request
    two_scara_collaboration::upper_boundary_changeResponse& response) {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "scara_upper_boundary_maintainer");
    ros::NodeHandle nh;

    // initialize publisher on topic "/scara_left_upper_boundary"

    // initialize publisher on topic "/scara_left_upper_boundary"

    // initialize service server of "/scara_left_upper_boundary_change"
    ros::ServiceServer left_change_server
        = nh.advertiseService("/scara_left_upper_boundary_change", leftChangeCallback);
    // initialize service server of "/scara_right_upper_boundary_change"
    ros::ServiceServer right_change_server
        = nh.advertiseService("/scara_right_upper_boundary_change", rightChangeCallback);




}



