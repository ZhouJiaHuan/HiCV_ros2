#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/visibility_control.hpp"
#include "face_detect/ultra_face.hpp"

namespace visual_composition
{
class FaceDetect: public rclcpp::Node
{
public:
    RCLCPP_COMPONENTS_PUBLIC
    explicit FaceDetect(const rclcpp::NodeOptions & options);

private:
    void imageSubCall(const sensor_msgs::msg::Image::UniquePtr msg);

    std::string mnnPath_;
    int width_;
    int height_;
    float scoreThresh_;
    float nmsThresh_;
    int numThreads_;
    bool draw_;
    std::mutex mtx_;
    std::shared_ptr<UltraFace> detector_;

    

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}