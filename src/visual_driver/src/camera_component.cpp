#include "visual_driver/camera_component.hpp"

using std::string;
using std::stringstream;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::image_encodings::BGR8;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


namespace visual_composition
{

Camera::Camera(const rclcpp::NodeOptions& options):
    Node("camera", options)
{
    declare_parameter<string>("camera_id", "0");
    declare_parameter<string>("info_path", "install/visual_driver/share/visual_driver/cfg/camera_info_640x480.yaml");
    declare_parameter<bool>("show", false);

    get_parameter("camera_id", camId_);
    get_parameter("info_path", infoPath_);
    get_parameter("show", show_);

    // ROS publishers
    streamPub_ = create_publisher<Image>("camera_stream", rclcpp::SensorDataQoS());
    infoPub_ = create_publisher<CameraInfo>("camera_info", rclcpp::SensorDataQoS());
    // Ros timer
    timer_ = create_wall_timer(1s, std::bind(&Camera::timerCallback, this));

    if (!this->initCamera())
    {
        RCLCPP_ERROR(get_logger(), "video stream open failed with camera index %i", camId_);
        rclcpp::shutdown();
    }
    videoLoop();
}


Camera::~Camera()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}


void Camera::timerCallback()
{
    // camera info
    if (cap_.isOpened() && infoPub_->get_subscription_count() > 0)
    {
        try
        {
            camInfoMsg_.header.stamp = this->now();
            infoPub_->publish(camInfoMsg_);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
        }
    }
}


bool Camera::initCamera()
{
    std::lock_guard<std::mutex> lock(mtx_);
    std::string cameraName;

    if (camera_calibration_parsers::readCalibration(infoPath_, cameraName, camInfoMsg_))
    {
        RCLCPP_INFO(get_logger(), "Parse camera info for '%s'", cameraName.c_str());
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Can not get camera info!");
        rclcpp::shutdown();
    }

    if (camId_.empty())
    {
        cap_ = cv::VideoCapture();
        return false;
    }
    else if (camId_.size() == 1)
    {
        stringstream ss;
        int camId;
        ss << camId_;
        ss >> camId;
        cap_ = cv::VideoCapture(camId);       
    }
    else
    {
       cap_ = cv::VideoCapture(camId_); 
    }

    try
    {
        width_ = camInfoMsg_.width;
        height_ = camInfoMsg_.height;
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), e.what());
        rclcpp::shutdown();
    }
    return cap_.isOpened();
}


void Camera::videoLoop()
{
    thread_ = std::thread([this]()
    {
        cv::Mat cvFrame;
        while (rclcpp::ok())
        {
            cap_ >> cvFrame;
            if (cvFrame.empty())
            {
                RCLCPP_WARN(get_logger(), "video stream empty!");
                break;
            }
            Image::UniquePtr imgMsg(new Image());
            imgMsg->header.stamp = this->now();
            imgMsg->header.frame_id = camInfoMsg_.header.frame_id;
            imgMsg->height = cvFrame.rows;
            imgMsg->width = cvFrame.cols;
            imgMsg->encoding = BGR8;
            imgMsg->is_bigendian = false;
            imgMsg->step = static_cast<Image::_step_type>(cvFrame.step);
            imgMsg->data.assign(cvFrame.datastart, cvFrame.dataend);
            // std::cout << imgMsg.get() << std::endl;
            if (streamPub_->get_subscription_count() > 0)
            {
                streamPub_->publish(std::move(imgMsg));
            }
            if (this->show_&& !cvFrame.empty())
            {
                cv::imshow("video stream", cvFrame);
                cv::waitKey(1);
            }
            std::this_thread::sleep_for(1ms);
        }
    }
    );
}

}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(visual_composition::Camera)