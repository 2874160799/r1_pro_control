#include "chassis_control/chassis_control.hpp"
#include <thread>
#include <mutex>

using namespace std;

ChassisControl::ChassisControl(const rclcpp::NodeOptions & options)
: Node("chassis_control_node", options) {
    RCLCPP_INFO(this->get_logger(), "chassis_control_node started");

    // 声明参数（带默认值）
    this->declare_parameter<std::string>("imu_topic", "/IMU_data");
    this->declare_parameter<std::string>("yaw_topic", "/YAW_data");
    this->declare_parameter<std::string>("cmd_vel_topic","/motion_target/target_speed_chassis");
    this->declare_parameter<int>("queue_size", 20);
    this->declare_parameter<double>("yaw_tolerance",0.02);//rad 0.02约1度
    this->declare_parameter<double>("k_p",1.5);


    // 获取参数值
    std::string imu_topic     = this->get_parameter("imu_topic").as_string();
    std::string yaw_topic     = this->get_parameter("yaw_topic").as_string();
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    int queue_size            = this->get_parameter("queue_size").as_int();

    // 创建订阅者
    // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     imu_topic, queue_size,
    //     std::bind(&ChassisControl::imuCallback, this, std::placeholders::_1)
    // );
    yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        yaw_topic,queue_size,
        std::bind(&ChassisControl::yawCallback, this, std::placeholders::_1)
    );

    // 创建发布者
    // yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(yaw_topic, queue_size);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic,queue_size);

    // RCLCPP_INFO(this->get_logger(), "Subscribed to IMU topic: %s", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to YAW topic: %s", yaw_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel to topic: %s", cmd_vel_topic.c_str());

}

// double ChassisControl::extractYawFromImu(const sensor_msgs::msg::Imu & imu_msg) {
//     tf2::Quaternion q{
//         imu_msg.orientation.x,
//         imu_msg.orientation.y,
//         imu_msg.orientation.z,
//         imu_msg.orientation.w
//     };

//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);
//     return yaw;
// }

// void ChassisControl::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
//     double yaw = this->extractYawFromImu(*msg);
//     std_msgs::msg::Float32 yaw_msg;
//     yaw_msg.data = static_cast<float>(yaw);
//     yaw_pub_->publish(yaw_msg);
//     // RCLCPP_INFO(this->get_logger(), "Yaw angle published: %.2f rad", yaw);
// }

double ChassisControl::normalizeAngle(double angle){
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void ChassisControl::yawCallback(const std_msgs::msg::Float32::SharedPtr msg){
    this->current_yaw_ = msg->data;
    RCLCPP_INFO(this->get_logger(),"yaw_data:%f",this->current_yaw_);
}

void ChassisControl::rotateAnyAngle(double angle_rad){
    if (control_running_){
        RCLCPP_WARN(this->get_logger(),"Rotation already in progress.");
        return;
    }
    
    double start_yaw;
    {
        std::lock_guard<std::mutex> lock(yaw_mutex_);
        start_yaw = this->current_yaw_;
    }
    target_yaw_ = this->normalizeAngle(start_yaw + angle_rad);
    control_running_ = true;
    //启动控制线程
    control_thread_ = std::thread(&ChassisControl::rotationTask,this); 
    control_thread_.detach();
}

void ChassisControl::rotationTask(){
    double tol = this->get_parameter("yaw_tolerance").as_double();
    double k_p = this->get_parameter("k_p").as_double();
    RCLCPP_INFO(this->get_logger(),"Rotating to target yaw: %.2f rad",this->target_yaw_); 
    rclcpp::Rate rate(50);

    while (rclcpp::ok()){
        
        double current;
        {
            std::lock_guard<std::mutex> lock(yaw_mutex_);
            current = current_yaw_;
        }
        double error = normalizeAngle(target_yaw_ - current);

        geometry_msgs::msg::TwistStamped cmd;

        if (fabs(error) < tol){
            cmd.twist.angular.z = 0.0;
            cmd.header.stamp = this->now();
            cmd_vel_pub_->publish(cmd);
            break;
        }
        cmd.twist.angular.z = k_p * error;
        if (cmd.twist.angular.z > 0.5) cmd.twist.angular.z = 0.5;
        if (cmd.twist.angular.z < -0.5) cmd.twist.angular.z = -0.5;

        cmd.header.stamp = this->now();
        cmd_vel_pub_->publish(cmd);
        rate.sleep();
    }
    control_running_ = false;
    RCLCPP_INFO(this->get_logger(), "Rotation finished.");
}

int main(int argc, char const *argv[])
{
    /* code */
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ChassisControl>();
    node->rotateAnyAngle(M_PI/2);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
