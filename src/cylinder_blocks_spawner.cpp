// spawn the red and blue cylinders on the conveyor belt
// and give them initial speed (by apply_body_wrench) to slide on conveyor

// ros communications:
    // spawn model through gazebo service: /gazebo/spawn_urdf_model
    // initialize cylinder speed: /gazebo/apply_body_wrench
    // get urdf file path of cylinder blocks from parameter server
    // publish all current blocks through topic: /current_cylinder_blocks

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <std_msgs/Int8MultiArray.h>

// int to string converter
std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cylinder_blocks_spawner");
    ros::NodeHandle nh;

    // service client for service /gazebo/spawn_urdf_model
    ros::ServiceClient spawn_model_client
        = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    gazebo_msgs::SpawnModel spawn_model_srv_msg;  // service message
    // service client for service /gazebo/apply_body_wrench
    ros::ServiceClient apply_wrench_client
        = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    gazebo_msgs::ApplyBodyWrench apply_wrench_srv_msg;  // service message

    // publisher for /current_cylinder_blocks
    ros::Publisher current_cylinder_publisher
        = nh.advertise<std_msgs::Int8MultiArray>("current_cylinder_blocks", 1);
    std_msgs::Int8MultiArray current_cylinder_msg;
    current_cylinder_msg.data.clear();

    // make sure /gazebo/spawn_urdf_model service is ready
    bool service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/spawn_urdf_model", true);
        ROS_INFO("waiting for spawn_urdf_model service");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("spawn_urdf_model service is ready");

    // make sure /gazebo/apply_body_wrench service is ready
    service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/apply_body_wrench", true);
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
    std::ifstream red_inXml, blue_inXml;
    std::stringstream red_strStream, blue_strStream;
    std::string red_xmlStr, blue_xmlStr;
    // red cylinder
    red_inXml.open(red_cylinder_path.c_str());
    red_strStream << red_inXml.rdbuf();
    red_xmlStr = red_strStream.str();
    // blue cylinder
    blue_inXml.open(blue_cylinder_path.c_str());
    blue_strStream << blue_inXml.rdbuf();
    blue_xmlStr = blue_strStream.str();

    // prepare the spawn model service message
    spawn_model_srv_msg.request.initial_pose.position.x = 4.0;
    spawn_model_srv_msg.request.initial_pose.position.z = 0.2;  // on the conveyor belt
    spawn_model_srv_msg.request.initial_pose.orientation.x = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.y = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.z = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.w = 1.0;
    spawn_model_srv_msg.request.reference_frame = "world";
    // reference_frame has to be empty, map or world. Otherwise gazebo will return error
    // http://answers.ros.org/question/65077/errors-while-applying-force-on-a-model/

    // prepare the apply body wrench service message
    // the exerted force and duration will decide the initial speed
    ros::Time time_temp(0, 0);
    ros::Duration duration_temp(0, 1000000);
    apply_wrench_srv_msg.request.wrench.force.x = -0.0012;
    apply_wrench_srv_msg.request.wrench.force.y = 0.0;
    apply_wrench_srv_msg.request.wrench.force.z = 0.0;
    apply_wrench_srv_msg.request.start_time = time_temp;
    apply_wrench_srv_msg.request.duration = duration_temp;
    apply_wrench_srv_msg.request.reference_frame = "world";

    // begin spawn cylinder blocks and give an initial speed on conveyor
    int i = 0;  // index the cylinder blocks
    while (ros::ok()) {
        std::string index_string = intToString(i);
        std::string model_name;
        int8_t color_index;  // 0 is red, 1 is blue

        // prepare spawn model service message
        spawn_model_srv_msg.request.initial_pose.position.y
            = (float)rand()/(float)(RAND_MAX) * 0.8 - 0.4;  // random between -0.4 to 0.4
        ROS_INFO_STREAM("y position of new cylinder: "
            << spawn_model_srv_msg.request.initial_pose.position.y);
        if ((rand() - RAND_MAX/2) > 0) {
            // then choose red cylinder
            color_index = 0;
            model_name = "red_cylinder_" + index_string;  // initialize model_name
            spawn_model_srv_msg.request.model_name = model_name;
            spawn_model_srv_msg.request.robot_namespace = "red_cylinder_" + index_string;
            spawn_model_srv_msg.request.model_xml = red_xmlStr;
        }
        else {
            // then choose blue cylinder
            color_index = 1;
            model_name = "blue_cylinder_" + index_string;
            spawn_model_srv_msg.request.model_name = model_name;
            spawn_model_srv_msg.request.robot_namespace = "blue_cylinder_" + index_string;
            spawn_model_srv_msg.request.model_xml = blue_xmlStr;
        }
        // call spawn model service
        bool call_service = spawn_model_client.call(spawn_model_srv_msg);
        if (call_service) {
            if (spawn_model_srv_msg.response.success) {
                ROS_INFO_STREAM(model_name << " has been spawned");
            }
            else {
                ROS_INFO_STREAM(model_name << " spawn failed");
            }
        }
        else {
            ROS_INFO("fail in first call");
            ROS_ERROR("fail to connect with gazebo server");
            return 0;
        }

        // prepare apply body wrench service message
        apply_wrench_srv_msg.request.body_name = model_name + "::base_link";
        // call apply body wrench service
        call_service = apply_wrench_client.call(apply_wrench_srv_msg);
        if (call_service) {
            if (apply_wrench_srv_msg.response.success) {
                ROS_INFO_STREAM(model_name << " speed initialized");
            }
            else {
                ROS_INFO_STREAM(model_name << " fail to initialize speed");
            }
        }
        else {
            ROS_ERROR("fail to connect with gazebo server");
            return 0;
        }

        // publish current cylinder blocks status, all cylinder blocks will be published
        // no matter if it's successfully spawned, or successfully initialized in speed
        current_cylinder_msg.data.push_back(color_index);
        current_cylinder_publisher.publish(current_cylinder_msg);

        // loop end, increase index by 1, add blank line
        i = i + 1;
        ROS_INFO_STREAM("");

        ros::spinOnce();
        ros::Duration(7.0).sleep();  // frequency control, spawn one cylinder in each loop
        // delay time decides density of the cylinders
    }
    return 0;
}



