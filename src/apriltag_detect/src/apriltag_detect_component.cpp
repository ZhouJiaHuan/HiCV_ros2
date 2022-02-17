#include "apriltag_detect/apriltag_detect_component.hpp"

using sensor_msgs::msg::Image;
using std::placeholders::_1;
using std::placeholders::_2;
using std::stringstream;
using namespace std::chrono_literals;

namespace visual_composition
{
ApriltagDetect::ApriltagDetect(const rclcpp::NodeOptions & options): Node("apriltag_detect", options)
{
    declare_parameter<std::string>("tag_family", "36h11");
    declare_parameter<bool>("show", true);
    get_parameter("tag_family", tagFamily_);
    get_parameter("show", draw_);
    

    if (tagFamily_ == "36h11")
    {
        tagDetector_ = std::make_shared<vpDetectorAprilTag>(vpDetectorAprilTag::TAG_36h11);
    }
    else if (tagFamily_ == "25h9")
    {
        tagDetector_ = std::make_shared<vpDetectorAprilTag>(vpDetectorAprilTag::TAG_25h9);
    }
    else if (tagFamily_ == "25h7")
    {
        tagDetector_ = std::make_shared<vpDetectorAprilTag>(vpDetectorAprilTag::TAG_25h7);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "unsupported apriltag family: %s!", tagFamily_.c_str());
        rclcpp::shutdown();
    }
    tagDetector_->setAprilTagNbThreads(2);
    tagDetector_->setAprilTagQuadDecimate(1.0);

    // topics
    imageSub_= create_subscription<Image>("camera_stream", rclcpp::SensorDataQoS(),
        std::bind(&ApriltagDetect::imageSubCall, this, _1));
}


void ApriltagDetect::imageSubCall(const Image::UniquePtr msg)
{
    if (msg->encoding.size() == 0 || msg->width == 0 || msg->height == 0)
    {
        RCLCPP_INFO(get_logger(), "invalid video frame!");
        return;
    }
   
    cv::Mat cvFrame(msg->height, msg->width, CV_8UC3, msg->data.data());
    cv::Mat cvFrameGray;
    cv::cvtColor(cvFrame, cvFrameGray, cv::COLOR_BGR2GRAY);
    vpImage<unsigned char> I;
    vpImageConvert::convert(cvFrameGray, I);
    auto ts = this->now();
    tagDetector_->detect(I);
    int tagNum = tagDetector_->getNbObjects();
    float timeMs = (float)(this->now() - ts).seconds() * 1000;
    
    if (draw_)
    {
        cv::Mat dstImage = cvFrame.clone();
        drawResult(dstImage);
        cv::imshow("apriltag detect result", dstImage);
        cv::waitKey(1);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "detect %i objects in %.2f ms", tagNum, timeMs);
    }
    
}


void ApriltagDetect::drawResult(cv::Mat &cvImg)
{
    for (size_t i=0; i<tagDetector_->getNbObjects(); i++)
    {
        // draw corner points
        auto poly = tagDetector_->getPolygon(i);
        cv::Point p1, p2;
        for (size_t i=0; i< poly.size()-1; i++)
        {
            p1 = cv::Point(poly[i].get_u(), poly[i].get_v());
            p2 = cv::Point(poly[i+1].get_u(), poly[i+1].get_v());
            cv::line(cvImg, p1, p2, cv::Scalar(255, 0, 0), 2);
        }
        p1 = cv::Point(poly[0].get_u(), poly[0].get_v());
        p2 = cv::Point(poly.back().get_u(), poly.back().get_v());
        cv::line(cvImg, p1, p2, cv::Scalar(255, 0, 0), 2);

        for (size_t i=0; i< poly.size(); i++)
        {
            stringstream ss;
            ss << i;
            cv::Point p(poly[i].get_u(), poly[i].get_v());
            cv::putText(cvImg, ss.str(), p, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255), 1);
        }
        
        // draw apriltag message
        std::string tagMsg = tagDetector_->getMessage(i);
        cv::putText(cvImg, tagMsg, p1, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255), 1);
    }
}
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(visual_composition::ApriltagDetect)