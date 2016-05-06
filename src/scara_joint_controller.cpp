// the joint controller for all four joint of two scara robots
// a PD controller read joint angle & rate, and apply joint torque

// the "Joint" class is copied from my previous project, with some minor changes
// one instance of this class to control one joint of the robots
// ros communication of the class:
    // service clients, /gazebo/get_joint_properties, /gazebo/apply_joint_effort
    // publish joint state, including posiiton, velocity, torque
    // subscribe to position command
    // service server for kp & kv tuning

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <string>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>
#include <two_scara_collaboration/kpkv_msg.h>

// class definition
class Joint {
public:
    Joint(ros::NodeHandle nh, std::string joint_name, double pos_cmd, double dt); // constructor
    ~Joint() {}; // destructor
    void getJointState();
    void jointTrqControl();
    void kpkvSetting(double kp, double kv);
private:
    // callback for the pos_cmd subscriber
    void posCmdCB(const std_msgs::Float64& pos_cmd_msg);
    // callback for kpkv service server
    bool kpkvCallback(two_scara_collaboration::kpkv_msgRequest& request
        , two_scara_collaboration::kpkv_msgResponse& response);
    // service clients
    ros::ServiceClient get_jnt_state_client;
    ros::ServiceClient set_trq_client;
    // publisher objects
    ros::Publisher trq_publisher;
    ros::Publisher vel_publisher;
    ros::Publisher pos_publisher;
    ros::Publisher joint_state_publisher;
    // subscriber object
    ros::Subscriber pos_cmd_subscriber;
    // gazebo/sensor messages
    gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
    gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg;
    sensor_msgs::JointState joint_state_msg;
    // position/velocity/torque messages to be published
    std_msgs::Float64 pos_msg; // position
    std_msgs::Float64 vel_msg; // velocity
    std_msgs::Float64 trq_msg; // torque
    // kpkv service server
    ros::ServiceServer kpkv_server;

    // control parameters
    double pos_cur; // current joint position
    double vel_cur; // current joint velocity
    double pos_cmd; // joint position from commander
    double pos_err; // error between pos_cmd and pos_cur
    double trq_cmd; // torque to be published
    double kp;
    double kv;
    // other parameters
    std::string joint_name;
};

Joint::Joint(ros::NodeHandle nh, std::string joint_name, double pos_cmd, double dt) {
    // initialize parameters
    this -> joint_name = joint_name;
    this -> pos_cmd = pos_cmd;  // give a initial position
    ros::Duration duration(dt);
    kp = 10.0;
    kv = 3.0;

    pos_cur = 0.0;

    // initialize gazebo clients
    get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
    set_trq_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    // initialize publisher objects
    // check if there is "::" symbol in the string, replace with "_"
    std::string joint_name_published = joint_name;
    std::size_t found = joint_name_published.find("::");
    if (found!=std::string::npos) {
        joint_name_published.replace(found, 2, "_");
    }
    pos_publisher = nh.advertise<std_msgs::Float64>(joint_name_published + "_pos", 1);
    vel_publisher = nh.advertise<std_msgs::Float64>(joint_name_published + "_vel", 1);
    trq_publisher = nh.advertise<std_msgs::Float64>(joint_name_published + "_trq", 1);
    joint_state_publisher = nh.advertise<sensor_msgs::JointState>(joint_name_published + "_states", 1); 
    // initialize subscriber object
    pos_cmd_subscriber = nh.subscribe(joint_name_published + "_pos_cmd", 1, &Joint::posCmdCB, this);
    // initialize kpkv service server
    kpkv_server = nh.advertiseService(joint_name_published + "_kpkv_service", &Joint::kpkvCallback, this);

    // set up get_joint_state_srv_msg
    get_joint_state_srv_msg.request.joint_name = joint_name;
    // set up effort_cmd_srv_msg
    effort_cmd_srv_msg.request.joint_name = joint_name;
    effort_cmd_srv_msg.request.effort = 0.0;
    effort_cmd_srv_msg.request.duration = duration;
    // set up joint_state_msg
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name.push_back(joint_name);
    joint_state_msg.position.push_back(0.0);
    joint_state_msg.velocity.push_back(0.0);
}

void Joint::posCmdCB(const std_msgs::Float64& pos_cmd_msg) {
    // too much information
    // ROS_INFO("received value of %s_pos_cmd is: %f", joint_name.c_str(), pos_cmd_msg.data); 
    pos_cmd = pos_cmd_msg.data;
}

void Joint::getJointState() {
    // get joint state
    get_jnt_state_client.call(get_joint_state_srv_msg);
    // publish joint position
    pos_cur = get_joint_state_srv_msg.response.position[0];
    pos_msg.data = pos_cur;
    pos_publisher.publish(pos_msg);
    // publish joint velocity
    vel_cur = get_joint_state_srv_msg.response.rate[0];
    vel_msg.data = vel_cur;
    vel_publisher.publish(vel_msg);
    // publish joint_state_msg
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.position[0] = pos_cur;
    joint_state_msg.velocity[0] = vel_cur;
    joint_state_publisher.publish(joint_state_msg);
}

// calculate joint torque, publish them, send to gazebo
void Joint::jointTrqControl() {
    pos_err = pos_cmd - pos_cur;
    // watch for periodicity
    // if (pos_err > M_PI)
    //     pos_err = pos_err - 2 * M_PI;
    // if (pos_err > M_PI)
    //     pos_err = pos_err + 2 * M_PI;
    // control algorithm in one line
    trq_cmd  = kp * pos_err - kv * vel_cur;
    // publish the torque message
    trq_msg.data = trq_cmd;
    trq_publisher.publish(trq_msg);
    // send torque command to gazebo
    effort_cmd_srv_msg.request.effort = trq_cmd;
    set_trq_client.call(effort_cmd_srv_msg);
    // make sure service call was successful
    bool result = effort_cmd_srv_msg.response.success;
    if (!result)
        ROS_WARN("service call to apply_joint_effort failed!");
}

void Joint::kpkvSetting(double kp, double kv) {
    this -> kp = kp;
    this -> kv = kv;
}

bool Joint::kpkvCallback(two_scara_collaboration::kpkv_msgRequest& request
    , two_scara_collaboration::kpkv_msgResponse& response) {
    ROS_INFO("kpkvCallback activated");
    kp = request.kp;
    kv = request.kv;
    // joint_name has to be converted to c_str() for ROS_INFO output
    ROS_INFO("%s: kp has been set to %f, kv has been set to %f", joint_name.c_str(), kp, kv);
    response.setting_is_done = true;
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "scara_joint_controller");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);

    // make sure apply_joint_effort service is ready
    bool service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
        ROS_INFO("waiting for apply_joint_effort service");
        half_sec.sleep();
    }
    ROS_INFO("apply_joint_effort service exists");
    // make sure get_joint_state_client service is ready
    service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/get_joint_properties",true);
        ROS_INFO("waiting for /gazebo/get_joint_properties service");
        half_sec.sleep();
    }
    ROS_INFO("/gazebo/get_joint_properties service exists");

    double dt = 0.01;  // sample time for the joint controller

    // instantiate 4 joint instances
    Joint scara_left_joint1(nh, "scara_robot_left::rotation1", -0.78, dt);
    Joint scara_left_joint2(nh, "scara_robot_left::rotation2", 2.1, dt);
    Joint scara_right_joint1(nh, "scara_robot_right::rotation1", -0.78, dt);
    Joint scara_right_joint2(nh, "scara_robot_right::rotation2", 2.1, dt);

    // set kp & kv here after being tuned
    scara_left_joint1.kpkvSetting(80, 16);
    scara_left_joint2.kpkvSetting(1, 0.2);
    scara_right_joint1.kpkvSetting(80, 16);
    scara_right_joint2.kpkvSetting(1, 0.2);

    ros::Rate rate_timer(1/dt);
    while (ros::ok()) {
        // get joint state(pos, vel) and publish them
        scara_left_joint1.getJointState();
        scara_left_joint2.getJointState();
        scara_right_joint1.getJointState();
        scara_right_joint2.getJointState();

        // calculate the torque for each joitn and publish them
        scara_left_joint1.jointTrqControl();
        scara_left_joint2.jointTrqControl();
        scara_right_joint1.jointTrqControl();
        scara_right_joint2.jointTrqControl();

        ros::spinOnce();  // update pos_cmd, kpkv
        rate_timer.sleep();  // sleep for the sample time
    }

}




