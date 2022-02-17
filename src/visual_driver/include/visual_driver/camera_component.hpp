#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/visibility_control.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_calibration_parsers/parse.h"


namespace visual_composition
{

class Camera: public rclcpp::Node
{
public:
    RCLCPP_COMPONENTS_PUBLIC
    explicit Camera(const rclcpp::NodeOptions& options);
    ~Camera();
    
private:
    bool initCamera();
    void videoLoop();
    void timerCallback();

    // topic
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr streamPub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub_;

    cv::VideoCapture cap_;
    std::string camId_;
    int width_;
    int height_;
    bool show_;
    std::string infoPath_;
    sensor_msgs::msg::CameraInfo camInfoMsg_;
    
    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    std::thread thread_;
    std::mutex mtx_;
};

}
