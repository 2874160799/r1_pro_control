
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "track/track.hpp"
#define LEFT 224;
using namespace std;

TrackNode::TrackNode(const rclcpp::NodeOptions & options) : Node("track_node",options){
    RCLCPP_INFO(this->get_logger(),"start TrackNode");
    int device_id = this->declare_parameter<int>("device_id",0);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/motion_target/target_speed_chassis",10);

    //开启摄像头
    cap_.open(device_id);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,camera_width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT,camera_height);
    if (!cap_.isOpened()){
        RCLCPP_ERROR(this->get_logger(),"Failed to open device id: %d!",device_id);
    } else {
        RCLCPP_INFO(this->get_logger(),"Successfully opened device id: %d!",device_id);
    }

    //创建滑动条窗口
    // cv::namedWindow("h_binary");
    // cv::namedWindow("s_binary");
    // cv::namedWindow("l_binary");
    // cv::namedWindow("binary");

    // hmin_ = 0; hmax_ = 180;
    // smin_ = 0; smax_ = 255;
    // lmin_ = 160; lmax_ = 255;

    // cv::createTrackbar("H min", "h_binary", &hmin_, 180);
    // cv::createTrackbar("H max", "h_binary", &hmax_, 180);
    // cv::createTrackbar("S min", "s_binary", &smin_, 255);
    // cv::createTrackbar("S max", "s_binary", &smax_, 255);
    // cv::createTrackbar("L min", "l_binary", &lmin_, 255);
    // cv::createTrackbar("L max", "l_binary", &lmax_, 255);

    //定时器读取摄像头内容
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30),
        std::bind(&TrackNode::camera_timer_callback,this)
    );
}

TrackNode::~TrackNode(){
    if (cap_.isOpened()){
        cap_.release();
    }
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "TrackNode destroyed, camera released");
}

void TrackNode::camera_timer_callback(){
    cv::Mat frame;
    if (!cap_.read(frame)){
        RCLCPP_WARN(this->get_logger(),"Failed to read frame from camera.");
        return;
    }
    if (!frame.empty()){
        cv::Mat binary = this->hls_process(frame);
        int error = this->showLine(binary,frame);
        // std::cout << "error:" << error << std::endl;

        cv::Mat small;
        cv::resize(binary, small, cv::Size(), 0.5, 0.5);  // 缩小一半
        cv::imshow("binary", small);
        cv::Mat small_frame;
        cv::resize(frame,small_frame,cv::Size(),0.5,0.5);
        cv::imshow("frame_with_line",small_frame);

        this->chassisControl(error);
        int key = cv::waitKey(1);
        if (key == 27){
            RCLCPP_INFO(this->get_logger(),"ESC pressed, shutting down.");
            rclcpp::shutdown();
        }
    }
}
//原始图像转hls图像
cv::Mat TrackNode::hls_process(const cv::Mat& oriFrame){
    cv::Mat hls;
    cv::cvtColor(oriFrame,hls,cv::COLOR_BGR2HLS);

    // 拆分通道
    std::vector<cv::Mat> channels;
    cv::split(hls,channels);
    cv::Mat h = channels[0];
    cv::Mat l = channels[1];
    cv::Mat s = channels[2];

    // 固定阈值
    int hmin = 0,   hmax = 180;
    int smin = 0,   smax = 255;
    int lmin = 0,   lmax = 140;

    // 阈值二值化
    cv::Mat h_binary, s_binary, l_binary, binary;
    cv::inRange(h, cv::Scalar(hmin), cv::Scalar(hmax), h_binary);
    cv::inRange(s, cv::Scalar(smin), cv::Scalar(smax), s_binary);
    cv::inRange(l, cv::Scalar(lmin), cv::Scalar(lmax), l_binary);

    // 三通道与操作，并取反
    cv::bitwise_and(h_binary, s_binary, binary);
    cv::bitwise_and(binary, l_binary, binary);
    cv::bitwise_not(binary, binary);

    return binary;
}

int TrackNode::showLine(const cv::Mat& binary,cv::Mat& frame){
    //取底部1/3为roi
    cv::Rect roi_rect(0,binary.rows*2/3,binary.cols,binary.rows/3);
    cv::Mat roi = binary(roi_rect);
    // cv::imshow("roi",roi);
    std::vector<std::vector<cv::Point>> contours;//定义存放轮廓的容器
    cv::findContours(roi,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);//在roi中找出最外层轮廓，存储在contours中

    if (!contours.empty()){
        //找出最大轮廓
        auto max_contour = *std::max_element(contours.begin(), contours.end(),
            [](const auto& a, const auto& b){
                return cv::contourArea(a) < cv::contourArea(b);
            });
        //绘制轮廓
        cv::drawContours(frame,std::vector<std::vector<cv::Point>>{max_contour},
                        -1,cv::Scalar(0,255,0),2,cv::LINE_8,
                        cv::noArray(),INT_MAX,cv::Point(0,binary.rows*2/3));
        //计算质心
        cv::Moments M = cv::moments(max_contour);
        if (M.m00 > 0){
            int cx = int(M.m10 / M.m00);
            int cy = int(M.m01 / M.m00);
            //绘制质心
            cv::circle(frame,cv::Point(cx,cy+binary.rows*2/3),
                        5,cv::Scalar(0,0,255),-1);
            //画中心线和误差线
            int center_x = frame.cols / 2;
            cv::line(frame,cv::Point(center_x,frame.rows),
                    cv::Point(cx,cy+binary.rows*2/3),
                    cv::Scalar(255,0,0),2);
            //返回偏差
            return cx - center_x - LEFT;
        }
    }
    else {
        std::cout << "Not Found Line!" << std::endl;
    }
    return 0;
}

//====================运动控制====================//
//测试案例，通过调节线的方向，控制轮子转动的方向
void TrackNode::chassisControl(int error){

    // double linearx_speed = 0.0;
    // double lineary_speed = -Kp * error;
    double proportional = Kp * error;
    integral += error;
    if (integral>1000) integral = 1000;
    if (integral<-1000) integral = -1000;
    double integral_term = Ki * integral;

    double derivative = error - prev_error;
    double derivative_term = Kd * derivative;
    prev_error = error;

    double lineary_speed = -(proportional + integral_term + derivative_term);
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->get_clock()->now();
    // cmd.header.frame_id = "base_link";
    // cmd.twist.linear.x = linearx_speed;
    cmd.twist.linear.y = lineary_speed;

    cmd_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(),"error=%d,linear_y=%.2f",error,lineary_speed);
}



int main(int argc, char const *argv[])
{
    /* code */
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TrackNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
