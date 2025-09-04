#ifndef CHASSIS_CONTROL__CHASSIS_CONTROL_HPP_
#define CHASSIS_CONTROL__CHASSIS_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include <thread>
#include <mutex>

class ChassisControl : public rclcpp::Node {
public:
    explicit ChassisControl(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    // double extractYawFromImu(const sensor_msgs::msg::Imu & imu_msg);
    void rotateAnyAngle(double angle_rad);



private:
    // void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void yawCallback(const std_msgs::msg::Float32::SharedPtr msg);

    double normalizeAngle(double angle);

    void rotationTask();
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    double current_yaw_,target_yaw_;
    bool control_running_ = false;

    std::thread control_thread_;
    std::mutex yaw_mutex_;
};

#endif  // CHASSIS_CONTROL__CHASSIS_CONTROL_HPP_
