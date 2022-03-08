#include "face_detect/face_detect_component.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using sensor_msgs::msg::Image;
using namespace std::chrono_literals;

const cv::Scalar CV_COLOR_BLUE(255, 0, 0);
const cv::Scalar CV_COLOR_GREEN(0, 255, 0);
const cv::Scalar CV_COLOR_RED(0, 0, 255);
const cv::Scalar CV_COLOR_YELLOW(0, 255, 255);
const cv::HersheyFonts CV_FONT = cv::FONT_HERSHEY_COMPLEX_SMALL;

namespace visual_composition
{
FaceDetect::FaceDetect(const rclcpp::NodeOptions & options): Node("face_detect", options)
{
    declare_parameter<std::string>("mnn_path", "install/face_detect/share/face_detect/model/slim-320.mnn");
    declare_parameter<int>("width", 320);
    declare_parameter<int>("height", 240);
    declare_parameter<float>("score_thresh", 0.6);
    declare_parameter<float>("nms_thresh", 0.3);
    declare_parameter<int>("num_threads", 2);
    declare_parameter<bool>("draw", true);

    get_parameter("mnn_path", mnnPath_);
    get_parameter("width", width_);
    get_parameter("height", height_);
    get_parameter("score_thresh", scoreThresh_);
    get_parameter("nms_thresh", nmsThresh_);
    get_parameter("num_threads", numThreads_);
    get_parameter("draw", draw_);

    detector_ = std::make_shared<UltraFace>(mnnPath_, width_, height_, numThreads_, scoreThresh_, nmsThresh_);
    imageSub_= create_subscription<Image>("camera_stream", rclcpp::SensorDataQoS(),
        std::bind(&FaceDetect::imageSubCall, this, _1));
}


void FaceDetect::imageSubCall(const Image::UniquePtr msg)
{
    if (msg->encoding.size() == 0 || msg->width == 0 || msg->height == 0)
    {
        RCLCPP_INFO(get_logger(), "invalid video frame!");
        return;
    }
    cv::Mat cvFrame(msg->height, msg->width, CV_8UC3, msg->data.data());

    std::vector<FaceInfo> faceInfo;
    auto ts = this->now();
    detector_->detect(cvFrame, faceInfo);
    float timeMs = (float)(this->now() - ts).seconds() * 1000;

    if (draw_)
    {
        cv::Mat dstImage = cvFrame.clone();
        for (auto face : faceInfo)
        {
            cv::Point pt1(face.x1, face.y1);
            cv::Point pt2(face.x2, face.y2);
            cv::rectangle(dstImage, pt1, pt2, cv::Scalar(0, 255, 0), 2);
            std::stringstream ss;
            ss << "score: " << std::setprecision(3) <<face.score;
            cv::putText(dstImage, ss.str(), pt1, CV_FONT, 1, CV_COLOR_RED);
        }
        // draw detection time
        std::stringstream detectTime;
        detectTime.precision(3);
        detectTime << "detect time: " << timeMs << " ms";
        cv::putText(dstImage, detectTime.str(), cv::Point(5, 20), CV_FONT, 1, CV_COLOR_RED);
        cv::imshow("face detect result", dstImage);
        cv::waitKey(1);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "detect %i faces in %.2f ms", (int)faceInfo.size(), timeMs);
    }
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(visual_composition::FaceDetect)