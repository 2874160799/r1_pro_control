#ifndef TRACK__TRACK_HPP_   
#define TRACK__TRACK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>


class TrackNode : public rclcpp::Node{
private:
    int camera_width = 1920;
    int camera_height = 1080;
    cv::VideoCapture cap_;

    rclcpp::TimerBase::SharedPtr timer_;//定时读取摄像头帧的定时器

    // HLS 阈值参数（滑动条绑定的变量）
    int hmin_, hmax_;
    int smin_, smax_;
    int lmin_, lmax_;

    

public:
    explicit TrackNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~TrackNode();

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    double Kp = 0.0008;
    double Ki = 0.0001;
    double Kd = 0.0002;

    double prev_error = 0.0;
    double integral = 0.0;

    void camera_timer_callback();

    cv::Mat hsv_process(const cv::Mat& oriFrame);

    cv::Mat hls_process(const cv::Mat& oriFrame);
    
    int showLine(const cv::Mat& binary,cv::Mat& frame);

    void chassisControl(int error);

};

#endif  // TRACK__TRACK_HPP_