#include <iostream>
#include <string>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/io/vpImageIo.h>
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp_components/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace visual_composition
{
class ApriltagDetect: public rclcpp::Node
{
public:
    RCLCPP_COMPONENTS_PUBLIC
    explicit ApriltagDetect(const rclcpp::NodeOptions & options);

private:
    void imageSubCall(const sensor_msgs::msg::Image::UniquePtr msg);
    void drawResult(cv::Mat &cvImg);

    std::shared_ptr<vpDetectorAprilTag> tagDetector_;
    std::string tagFamily_;

    std::mutex mtx_;
    bool draw_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    
    // timer
    rclcpp::TimerBase::SharedPtr timer_;
};
}