#ifndef TRACK__TRACK_HPP_   
#define TRACK__TRACK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <geometry_msgs/msg/point.hpp>

#include "track/msg/skeleton_points.hpp" 


class TrackNode : public rclcpp::Node{
private:
    int camera_width = 1920;
    int camera_height = 1080;
    cv::VideoCapture cap_leftfront_;
    cv::VideoCapture cap_rightfront_;
    cv::VideoCapture cap_left_;
    cv::VideoCapture cap_right_;
    cv::VideoCapture cap_back_;

    rclcpp::TimerBase::SharedPtr timer_;//定时读取摄像头帧的定时器
    rclcpp::TimerBase::SharedPtr rightFrontTimer_;//定时读取右前摄像头的定时器
    rclcpp::TimerBase::SharedPtr leftTimer_;
    rclcpp::TimerBase::SharedPtr rightTimer_;
    rclcpp::TimerBase::SharedPtr backTimer_;

    rclcpp::TimerBase::SharedPtr displayTimer_;

    rclcpp::Publisher<track::msg::SkeletonPoints>::SharedPtr skeleton_pub_;



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

    cv::Mat frame_leftfront_,frame_rightfront_,frame_left_,frame_right_,frame_back_;
    std::mutex mtx_;

    bool initCamera(cv::VideoCapture& cap,int device_id,const std::string& name);

    void camera_timer_callback();
    void cameraRightFront_timer_callbcak();
    void cameraLeft_timer_callback();
    void cameraRight_timer_callback();
    void cameraBack_timer_callback();

    void display_timer_callback();

    cv::Mat hsv_process(const cv::Mat& oriFrame);

    cv::Mat hls_process(const cv::Mat& oriFrame);
    
    int showLine(const cv::Mat& binary,cv::Mat& frame);

    void saveContourToJson(const std::vector<cv::Point>& contour, const std::string& filename);

    void chassisControl(int error);

    void publishSkeletonPoints(const std::vector<cv::Point> &points,
                                    int roi_x, int roi_y);

};

#endif  // TRACK__TRACK_HPP_