// spawn the red and blue cylinders on the conveyor belt
// and give them initial speed (by apply_body_wrench) to slide on conveyor

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <gazebo_msgs/SapwnModel.h>
#include <gazebo_msgs/ApplyBodyWrench.h>


// int to string converter
std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cylinder_objects_spawner");
    ros::NodeHandle nh;

    // service client for service /gazebo/spawn_urdf_model
    ros::ServiceClient spawn_model_client;
        = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    gazebo_msgs::SpawnModel spawn_model_srv_msg;  // service message
    // service client for service /gazebo/apply_body_wrench
    ros::ServiceClient apply_wrench_client;
        = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    gazebo_msgs::ApplyBodyWrench apply_wrench_srv_msg;  // service message

    // make sure /gazebo/spawn_urdf_model service is ready
    bool service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/spawn_urdf_model",true);
        ROS_INFO("waiting for spawn_urdf_model service");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("spawn_urdf_model service is ready");

    // make sure /gazebo/apply_body_wrench service is ready
    service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/apply_body_wrench",true);
        ROS_INFO("waiting for apply_body_wrench service");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("apply_body_wrench service is ready");

    // get file path of cylinder blocks from parameter server
    std::string red_cylinder_path;
    std::string blue_cylinder_path;
    bool get_red_path, get_blue_path;
    get_red_path = nh.getParam("/red_cylinder_path", red_cylinder_path);
    get_blue_path = nh.getParam("/blue_cylinder_path", blue_cylinder_path);
    if (!(get_red_path && get_blue_path))
        return 0;  // return if fail to get parameters

    // prepare the xml for service call, read urdf into string
    std::ifstream inXml;
    std::stringstream strStream;
    std::string red_xmlStr, blue_xmlStr;
    // red cylinder
    inXml.open(red_cylinder_path.c_str());
    strStream  << inXml.rdbuf();
    red_xmlStr = strStream.str();
    // blue cylinder
    inXml.open(blue_cylinder_path.c_str());
    strStream  << inXml.rdbuf();
    blue_xmlStr = strStream.str();

    // prepare the spawn model service message
    spawn_model_srv_msg.request.initial_pose.position.x = 4.0;
    spawn_model_srv_msg.request.initial_pose.position.z = 0.2;  // on the conveyor belt
    spawn_model_srv_msg.request.initial_pose.orientation.x = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.y = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.z = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.w = 1.0;
    spawn_model_srv_msg.request.reference_frame = "world";

    // prepare the apply body wrench service message
    apply_wrench_srv_msg.request.wrench.force.x = -0.002;
    apply_wrench_srv_msg.request.start_time = 0;
    apply_wrench_srv_msg.request.duration = 1000000;

    // begin spawn cylinder blocks and slide on conveyor
    while (ros::ok()) {
        
    }

}



