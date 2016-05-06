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
bool g_left_in_action;
double g_left_upper_boundary;
bool g_right_in_action;
double g_right_upper_boundary;

// callback to update global upper boundary data for left scara
bool leftChangeCallback(two_scara_collaboration::upper_boundary_changeRequest& request
    , two_scara_collaboration::upper_boundary_changeResponse& response) {
    if (request.in_action_change) {
        g_left_in_action = request.in_action;
    }
    if (request.upper_boundary_change) {
        g_left_upper_boundary = request.upper_boundary;
    }
    response.change_is_done = true;
    return response.change_is_done;
}

// callback to update global upper boundary data for right scara
bool rightChangeCallback(two_scara_collaboration::upper_boundary_changeRequest& request
    , two_scara_collaboration::upper_boundary_changeResponse& response) {
    if (request.in_action_change) {
        g_right_in_action = request.in_action;
    }
    if (request.upper_boundary_change) {
        g_right_upper_boundary = request.upper_boundary;
    }
    response.change_is_done = true;
    return response.change_is_done;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "scara_upper_boundary_maintainer");
    ros::NodeHandle nh;

    // initialize global data
    g_left_in_action = false;
    g_left_upper_boundary = 0.6;
    g_right_in_action = false;
    g_right_upper_boundary = -0.6;

    // initialize publisher on topic "/scara_left_upper_boundary"
    ros::Publisher scara_left_upper_boundary_publisher
        = nh.advertise<two_scara_collaboration::scara_upper_boundary>("/scara_left_upper_boundary", 1);
    two_scara_collaboration::scara_upper_boundary scara_left_upper_boundary_msg;
    // initialize publisher on topic "/scara_left_upper_boundary"
    ros::Publisher scara_right_upper_boundary_publisher
        = nh.advertise<two_scara_collaboration::scara_upper_boundary>("/scara_right_upper_boundary", 1);
    two_scara_collaboration::scara_upper_boundary scara_right_upper_boundary_msg;
    // initialize service server of "/scara_left_upper_boundary_change"
    ros::ServiceServer left_change_server
        = nh.advertiseService("/scara_left_upper_boundary_change", leftChangeCallback);
    // initialize service server of "/scara_right_upper_boundary_change"
    ros::ServiceServer right_change_server
        = nh.advertiseService("/scara_right_upper_boundary_change", rightChangeCallback);

    // publish loop
    ros::Rate rate_timer(50);
    while (ros::ok()) {
        // prepare published msg
        scara_left_upper_boundary_msg.in_action = g_left_in_action;
        scara_left_upper_boundary_msg.upper_boundary = g_left_upper_boundary;
        scara_right_upper_boundary_msg.in_action = g_right_in_action;
        scara_right_upper_boundary_msg.upper_boundary = g_right_upper_boundary;
        // publish the messages
        scara_left_upper_boundary_publisher.publish(scara_left_upper_boundary_msg);
        scara_right_upper_boundary_publisher.publish(scara_right_upper_boundary_msg);

        rate_timer.sleep();
        ros::spinOnce();
    }
}


