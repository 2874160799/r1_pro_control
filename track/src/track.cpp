#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "track/track.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

#define LEFT 224-9
using namespace std;

// 多线程进入摄像头回调
TrackNode::TrackNode(const rclcpp::NodeOptions & options) : Node("track_node",options){
    RCLCPP_INFO(this->get_logger(),"start TrackNode");
    int device_leftfront = this->declare_parameter<int>("device_leftfront",0);
    int device_rightfront = this->declare_parameter<int>("device_rightfront",1);
    int device_left = this->declare_parameter<int>("device_left",2);
    int device_right = this->declare_parameter<int>("device_right",3);
    int device_back = this->declare_parameter<int>("device_back",4);

    hmin_ = this->declare_parameter<int>("hmin", 0);
    hmax_ = this->declare_parameter<int>("hmax", 180);
    smin_ = this->declare_parameter<int>("smin", 0);
    smax_ = this->declare_parameter<int>("smax", 255);
    lmin_ = this->declare_parameter<int>("lmin", 0);
    lmax_ = this->declare_parameter<int>("lmax", 130);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/motion_target/target_speed_chassis",10);

    skeleton_pub_ = this->create_publisher<track::msg::SkeletonPoints>("skeleton_points", 10);


    initCamera(cap_leftfront_,device_leftfront,"leftfront");
    initCamera(cap_rightfront_,device_rightfront,"rightfront");
    initCamera(cap_left_,device_left,"left");
    initCamera(cap_right_,device_right,"right");
    initCamera(cap_back_,device_back,"back");

    //创建滑动条窗口
    // cv::namedWindow("h_binary");
    // cv::namedWindow("s_binary");
    cv::namedWindow("l_binary");
    // cv::namedWindow("binary");

    // hmin_ = 0; hmax_ = 180;
    // smin_ = 0; smax_ = 255;
    // lmin_ = 160; lmax_ = 255;

    cv::createTrackbar("H min", "h_binary", &hmin_, 180);
    cv::createTrackbar("H max", "h_binary", &hmax_, 180);
    cv::createTrackbar("S min", "s_binary", &smin_, 255);
    cv::createTrackbar("S max", "s_binary", &smax_, 255);
    cv::createTrackbar("L min", "l_binary", &lmin_, 255);
    cv::createTrackbar("L max", "l_binary", &lmax_, 255);

    //定时器读取左前摄像头内容
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30),
        std::bind(&TrackNode::camera_timer_callback,this)
    );
    // //定时器读取右前摄像头内容
    // rightFrontTimer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(30),
    //     std::bind(&TrackNode::cameraRightFront_timer_callbcak,this)
    // );
    // //定时器读取左摄像头内容
    // leftTimer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(30),
    //     std::bind(&TrackNode::cameraLeft_timer_callback,this)
    // );
    // rightTimer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(30),
    //     std::bind(&TrackNode::cameraRight_timer_callback,this)
    // );
    // backTimer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(30),
    //     std::bind(&TrackNode::cameraBack_timer_callback,this)
    // );    
    

}

TrackNode::~TrackNode(){
    if (cap_leftfront_.isOpened()){
        cap_leftfront_.release();
    }
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "TrackNode destroyed, camera released");
}


bool TrackNode::initCamera(cv::VideoCapture& cap,int device_id,const std::string& name){
    cap.open(device_id);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,camera_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,camera_height);
    if (!cap.isOpened()){
        RCLCPP_ERROR(this->get_logger(),"Failed to open %s camera id: %d!",name.c_str(),device_id);
        return false;
    }
    RCLCPP_INFO(this->get_logger(),"Successfully opened %s camera id: %d!",name.c_str(),device_id);
    return true;
}


//左前摄像头回调函数
void TrackNode::camera_timer_callback(){
    cv::Mat frame;
    if (!cap_leftfront_.read(frame)){
        RCLCPP_WARN(this->get_logger(),"Failed to read frame from LeftFrontCamera.");
        return;
    }
    if (!frame.empty()){
        cv::Mat binary = this->hls_process(frame);
        int error = this->showLine(binary,frame);
        std::cout << "error:" << error << std::endl;

        cv::Mat small;
        cv::resize(binary, small, cv::Size(), 0.5, 0.5);  // 缩小一半
        cv::imshow("binary", small);
        cv::Mat small_frame;
        cv::resize(frame,small_frame,cv::Size(),0.5,0.5);
        cv::imshow("camera_leftfront",small_frame);

        // this->chassisControl(error);
        int key = cv::waitKey(1);
        if (key == 27){
            RCLCPP_INFO(this->get_logger(),"ESC pressed, shutting down.");
            rclcpp::shutdown();
        }
    }
}


//右前摄像头回调函数
void TrackNode::cameraRightFront_timer_callbcak(){
    cv::Mat frame;
    if (!cap_rightfront_.read(frame)){
        RCLCPP_WARN(this->get_logger(),"Failed to read frame from RightFrontCamera.");
        return;
    }
    if (!frame.empty()){
        // cv::Mat binary = this->hls_process(frame);
        cv::Mat small_frame;
        cv::resize(frame,small_frame,cv::Size(),0.5,0.5);
        cv::imshow("camera_rightfront",small_frame);
    }
}

//左摄像头回调函数
void TrackNode::cameraLeft_timer_callback(){
    cv::Mat frame;
    if (!cap_left_.read(frame)){
        RCLCPP_WARN(this->get_logger(),"Failed to read frame from LeftCamera.");
        return;
    }
    if (!frame.empty()){
        // cv::Mat binary = this->hls_process(frame);
        cv::Mat small_frame;
        cv::resize(frame,small_frame,cv::Size(),0.5,0.5);
        cv::imshow("camera_left",small_frame);
    }
}
//右摄像头回调函数
void TrackNode::cameraRight_timer_callback(){
    cv::Mat frame;
    if (!cap_right_.read(frame)){
        RCLCPP_WARN(this->get_logger(),"Failed to read frame from RightCamera.");
        return;
    }
    if (!frame.empty()){
        // cv::Mat binary = this->hls_process(frame);
        cv::Mat small_frame;
        cv::resize(frame,small_frame,cv::Size(),0.5,0.5);
        cv::imshow("camera_right",small_frame);
    }
}
//后摄像头回调函数
void TrackNode::cameraBack_timer_callback(){
    cv::Mat frame;
    if (!cap_back_.read(frame)){
        RCLCPP_WARN(this->get_logger(),"Failed to read frame from BackCamera.");
        return;
    }
    if (!frame.empty()){
        // cv::Mat binary = this->hls_process(frame);
        cv::Mat small_frame;
        cv::resize(frame,small_frame,cv::Size(),0.5,0.5);
        cv::imshow("camera_back",small_frame);
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



    // 阈值二值化
    cv::Mat h_binary, s_binary, l_binary, binary;
    cv::inRange(h, cv::Scalar(hmin_), cv::Scalar(hmax_), h_binary);
    cv::inRange(s, cv::Scalar(smin_), cv::Scalar(smax_), s_binary);
    cv::inRange(l, cv::Scalar(lmin_), cv::Scalar(lmax_), l_binary);

    // 三通道与操作，并取反
    cv::bitwise_and(h_binary, s_binary, binary);
    cv::bitwise_and(binary, l_binary, binary);
    cv::bitwise_not(binary, binary);

    return binary;
}

int TrackNode::showLine(const cv::Mat& binary, cv::Mat& frame) {
    int img_width = binary.cols;
    int img_height = binary.rows;

    // ROI: 底部1/3，中间2/3宽
    int roi_y = img_height * 2 / 3;
    int roi_h = img_height / 3;
    int roi_x = img_width / 6;
    int roi_w = img_width * 2 / 3;

    cv::Rect roi_rect(roi_x, roi_y, roi_w, roi_h);
    cv::Mat roi = binary(roi_rect);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        std::cout << "Not Found Line!" << std::endl;
        return 0;
    }

    // 取最大轮廓
    auto max_contour = *std::max_element(contours.begin(), contours.end(),
        [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });

    // 绘制轮廓
    cv::drawContours(frame, std::vector<std::vector<cv::Point>>{max_contour},
                     -1, cv::Scalar(0,255,0), 2, cv::LINE_8,
                     cv::noArray(), INT_MAX, cv::Point(roi_x, roi_y));

    // 创建填充掩码
    cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{max_contour}, cv::Scalar(255));

    // 提取中心线（骨架）
    cv::Mat skeleton = cv::Mat::zeros(mask.size(), CV_8UC1);
    cv::Mat temp;
    cv::Mat eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    do {
        cv::erode(mask, eroded, element);
        cv::dilate(eroded, temp, element);
        cv::subtract(mask, temp, temp);
        cv::bitwise_or(skeleton, temp, skeleton);
        eroded.copyTo(mask);
    } while (cv::countNonZero(mask) != 0);

    // 将中心线转换回原图坐标并绘制
    std::vector<cv::Point> skeleton_points;
    cv::findNonZero(skeleton, skeleton_points);
    for (const auto& pt : skeleton_points) {
        cv::circle(frame, cv::Point(pt.x + roi_x, pt.y + roi_y), 2, cv::Scalar(0, 0, 255), -1);
    }
    publishSkeletonPoints(skeleton_points, roi_x, roi_y);

    return 0;
}



void TrackNode::saveContourToJson(const std::vector<cv::Point>& contour, const std::string& filename) {
    nlohmann::json j;
    j["contour"] = nlohmann::json::array();

    for (auto& p : contour) {
        j["contour"].push_back({ {"x", p.x}, {"y", p.y} });
    }

    std::ofstream file(filename);
    if (file.is_open()) {
        file << j.dump(4); // 缩进 4 个空格，更好阅读
        file.close();
        std::cout << "Saved contour to " << filename << std::endl;
    } else {
        std::cerr << "Failed to open file " << filename << std::endl;
    }
}

void TrackNode::publishSkeletonPoints(const std::vector<cv::Point> &points,
                                      int roi_x, int roi_y)
{
    track::msg::SkeletonPoints msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link"; // 视情况修改

    msg.points.reserve(points.size());
    for (const auto &pt : points) {
        geometry_msgs::msg::Point p;
        p.x = pt.x + roi_x;
        p.y = pt.y + roi_y;
        p.z = 0.0;
        msg.points.push_back(p);
    }

    skeleton_pub_->publish(msg);
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
