
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RealSensePublisher : public rclcpp::Node
{
public:
    RealSensePublisher() : Node("realsense_publisher")
    {
        // Declare parameters
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("fps", 30);
        
        // Get parameters
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();
        
        // Create publishers
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 10);
        
        // Configure RealSense pipeline
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
        
        try {
            // Start pipeline
            pipe_profile_ = pipe_.start(cfg);
            RCLCPP_INFO(this->get_logger(), "RealSense D455 camera started successfully");
            
            // Get camera intrinsics for camera_info
            auto stream = pipe_profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            auto intrinsics = stream.get_intrinsics();
            
            // Setup camera info message
            setupCameraInfo(intrinsics);
            
            // Create timer for publishing at specified rate
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / fps),
                std::bind(&RealSensePublisher::publishFrames, this));
                
        } catch (const rs2::error &e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
            throw;
        }
    }
    
    ~RealSensePublisher()
    {
        pipe_.stop();
        RCLCPP_INFO(this->get_logger(), "RealSense pipeline stopped");
    }

private:
    void setupCameraInfo(const rs2_intrinsics& intrinsics)
    {
        camera_info_msg_.width = intrinsics.width;
        camera_info_msg_.height = intrinsics.height;
        camera_info_msg_.distortion_model = "plumb_bob";
        
        // Intrinsic camera matrix
        camera_info_msg_.k = {
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0
        };
        
        // Distortion coefficients
        camera_info_msg_.d = {
            intrinsics.coeffs[0], intrinsics.coeffs[1], 
            intrinsics.coeffs[2], intrinsics.coeffs[3], 
            intrinsics.coeffs[4]
        };
        
        // Rectification matrix (identity for unrectified)
        camera_info_msg_.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
        
        // Projection matrix
        camera_info_msg_.p = {
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        };
    }
    
    void publishFrames()
    {
        rs2::frameset frames;
        
        // Wait for frames with timeout
        if (!pipe_.poll_for_frames(&frames)) {
            return;
        }
        
        auto now = this->now();
        
        // Process color frame
        rs2::video_frame color_frame = frames.get_color_frame();
        if (color_frame) {
            publishColorImage(color_frame, now);
            publishCameraInfo(now);
        }
        
        // Process depth frame
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (depth_frame) {
            publishDepthImage(depth_frame, now);
        }
    }
    
    void publishColorImage(const rs2::video_frame& frame, const rclcpp::Time& timestamp)
    {
        // Create OpenCV matrix from frame data
        cv::Mat color_mat(cv::Size(frame.get_width(), frame.get_height()),
                         CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        
        // Convert to ROS message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_mat).toImageMsg();
        msg->header.stamp = timestamp;
        msg->header.frame_id = "camera_color_optical_frame";
        
        color_pub_->publish(*msg);
    }
    
    void publishDepthImage(const rs2::depth_frame& frame, const rclcpp::Time& timestamp)
    {
        // Create OpenCV matrix from depth data
        cv::Mat depth_mat(cv::Size(frame.get_width(), frame.get_height()),
                         CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        
        // Convert to ROS message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth_mat).toImageMsg();
        msg->header.stamp = timestamp;
        msg->header.frame_id = "camera_depth_optical_frame";
        
        depth_pub_->publish(*msg);
    }
    
    void publishCameraInfo(const rclcpp::Time& timestamp)
    {
        camera_info_msg_.header.stamp = timestamp;
        camera_info_msg_.header.frame_id = "camera_color_optical_frame";
        camera_info_pub_->publish(camera_info_msg_);
    }
    
    // RealSense pipeline
    rs2::pipeline pipe_;
    rs2::pipeline_profile pipe_profile_;
    
    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Camera info message
    sensor_msgs::msg::CameraInfo camera_info_msg_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<RealSensePublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}