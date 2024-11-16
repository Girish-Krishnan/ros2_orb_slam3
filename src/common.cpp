/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

// Includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "System.h" // Include ORB-SLAM3 System header
#include <Eigen/Dense> // Include Eigen for matrix operations

using std::placeholders::_1;

class MonocularMode : public rclcpp::Node
{
public:
    MonocularMode(); // Constructor declaration
    ~MonocularMode(); // Destructor

private:
    // Class variables
    std::string experimentConfig;
    double timeStep;
    ORB_SLAM3::System* pAgent;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    bool bSettingsFromPython;

    void experimentSetting_callback(const std_msgs::msg::String& msg);
    void Timestep_callback(const std_msgs::msg::Float64& time_msg);
    void Img_callback(const sensor_msgs::msg::Image& msg);
    void initializeVSLAM(const std::string& configString);
};

// Constructor definition
MonocularMode::MonocularMode()
    : Node("mono_slam_cpp")
{
    // Parameter declaration
    this->declare_parameter<std::string>("settings_name", "EuRoC");
    this->declare_parameter<std::string>("image_seq", "NULL");

    // Subscription to experiment settings from Python node
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/mono_py_driver/experiment_settings", 10, std::bind(&MonocularMode::experimentSetting_callback, this, _1));
    
    // Subscription to image messages
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/mono_py_driver/img_msg", 10, std::bind(&MonocularMode::Img_callback, this, _1));
    
    // Subscription to timestep messages
    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mono_py_driver/timestep_msg", 10, std::bind(&MonocularMode::Timestep_callback, this, _1));
    
    // Publisher for acknowledging the receipt of experiment settings
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/mono_py_driver/exp_settings_ack", 10);
    
    // Publisher for the camera pose
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/camera_pose", 10);

    // Set initial flags and variables
    bSettingsFromPython = false;
    pAgent = nullptr; // Initialize pointer to ORB SLAM3 System object
}

// Destructor definition
MonocularMode::~MonocularMode()
{
    if (pAgent != nullptr)
    {
        delete pAgent;
    }
}

// Callback to handle experiment settings
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg)
{
    experimentConfig = msg.data;
    RCLCPP_INFO(this->get_logger(), "Received experiment settings: '%s'", experimentConfig.c_str());

    if (!bSettingsFromPython)
    {
        bSettingsFromPython = true;
        std_msgs::msg::String ack_msg;
        ack_msg.data = "ACK";
        configAck_publisher_->publish(ack_msg);
        RCLCPP_INFO(this->get_logger(), "Sent ACK for experiment settings.");

        initializeVSLAM(experimentConfig);
    }
}

// Callback to handle the timestep
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg)
{
    timeStep = time_msg.data;
}

// Callback to process incoming images
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    if (pAgent != nullptr)
    {
        // Convert ROS image message to OpenCV image
        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // Track the image using ORB-SLAM3 and get the camera pose
        Sophus::SE3f pose_SE3 = pAgent->TrackMonocular(img, timeStep);

        if (pose_SE3.translation().norm() > 1e-6) // Check if the pose is non-zero
        {
            // Publish the pose
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "camera_link";

            Eigen::Quaternionf q(pose_SE3.unit_quaternion());
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            pose_msg.pose.position.x = pose_SE3.translation().x();
            pose_msg.pose.position.y = pose_SE3.translation().y();
            pose_msg.pose.position.z = pose_SE3.translation().z();

            pose_publisher_->publish(pose_msg);
        }
    }
}

// Initialize ORB-SLAM3 system with given settings
void MonocularMode::initializeVSLAM(const std::string& configString)
{
    // Define the path to the ORB-SLAM3 vocabulary and settings file
    std::string vocFilePath = "/home/erl-xarm6/nikola_moveit_ws/Nikola_ERL_Robot_Manipulation_Moveit/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin"; // Replace with your actual path
    std::string settingsFilePath = "/home/erl-xarm6/nikola_moveit_ws/Nikola_ERL_Robot_Manipulation_Moveit/src/ros2_orb_slam3/orb_slam3/config/Monocular/" + configString + ".yaml"; // Replace with your actual path

    // Check if files exist
    if (!std::ifstream(vocFilePath).good())
    {
        RCLCPP_ERROR(this->get_logger(), "Vocabulary file not found at: %s", vocFilePath.c_str());
        return;
    }

    if (!std::ifstream(settingsFilePath).good())
    {
        RCLCPP_ERROR(this->get_logger(), "Settings file not found at: %s", settingsFilePath.c_str());
        return;
    }

    // Initialize ORB-SLAM3
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, ORB_SLAM3::System::MONOCULAR, true);

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 system initialized.");
}

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MonocularMode>(); // Should now link correctly
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
