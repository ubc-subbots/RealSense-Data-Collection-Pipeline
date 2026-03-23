#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>

class RealSensePublisher : public rclcpp::Node
{
public:
    RealSensePublisher() : Node("realsense_publisher")
    {
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("fps", 30);
        this->declare_parameter<bool>("enable_imu", true);
        
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();
        bool enable_imu = this->get_parameter("enable_imu").as_bool();
        int imu_hz = 200;
        
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/camera/imu", 100);
        
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
        
        if (enable_imu) {
            try {
                cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, imu_hz);
                cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, imu_hz);
                RCLCPP_INFO(this->get_logger(), "IMU streams enabled at %d HZ ", imu_hz);
            } catch (const rs2::error &e) {
                RCLCPP_WARN(this->get_logger(), "Could not enable IMU: %s", e.what());
            }
        }
        
        try {
            pipe_.start(cfg, [this](const rs2::frame& frame) {
                this->frameCallback(frame);
            });
            
            RCLCPP_INFO(this->get_logger(), "RealSense camera started successfully");
            
            auto pipe_profile = pipe_.get_active_profile();
            auto stream = pipe_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            auto intrinsics = stream.get_intrinsics();
            
            setupCameraInfo(intrinsics);
            
            RCLCPP_INFO(this->get_logger(), "Publisher ready - Camera: %d Hz, IMU: %d Hz", fps, imu_hz);
                
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
    
    // This callback is called by RealSense SDK whenever a frame arrives
    void frameCallback(const rs2::frame& frame)
    {
        auto now = this->now();
        
        if (auto fs = frame.as<rs2::frameset>()) {
            processFrameset(fs, now);
        }
        else if (auto motion = frame.as<rs2::motion_frame>()) {
            processMotionFrame(motion, now);
        }
    }
    
    void processFrameset(const rs2::frameset& frames, const rclcpp::Time& timestamp)
    {
        rs2::video_frame color_frame = frames.get_color_frame();
        if (color_frame) {
            publishColorImage(color_frame, timestamp);
            publishCameraInfo(timestamp);
        }
        
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (depth_frame) {
            publishDepthImage(depth_frame, timestamp);
        }
    }
    
    void processMotionFrame(const rs2::motion_frame& motion, const rclcpp::Time& timestamp)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        
        if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL) {
            latest_accel_ = std::make_shared<rs2::motion_frame>(motion);
        }
        else if (motion.get_profile().stream_type() == RS2_STREAM_GYRO) {
            latest_gyro_ = std::make_shared<rs2::motion_frame>(motion);
        }
        
        if (latest_accel_ && latest_gyro_) {
            publishIMU(*latest_accel_, *latest_gyro_, timestamp);
        }
    }
    
    void publishIMU(const rs2::motion_frame& accel, const rs2::motion_frame& gyro, 
                    const rclcpp::Time& timestamp)
    {
        sensor_msgs::msg::Imu imu_msg;
        
        imu_msg.header.stamp = timestamp;
        imu_msg.header.frame_id = "camera_imu_optical_frame";
        
        rs2_vector accel_data = accel.get_motion_data();
        rs2_vector gyro_data = gyro.get_motion_data();
        
        imu_msg.linear_acceleration.x = accel_data.x;
        imu_msg.linear_acceleration.y = accel_data.y;
        imu_msg.linear_acceleration.z = accel_data.z;
        
        imu_msg.angular_velocity.x = gyro_data.x;
        imu_msg.angular_velocity.y = gyro_data.y;
        imu_msg.angular_velocity.z = gyro_data.z;
        
        // Orientation is not provided by RealSense IMU
        imu_msg.orientation_covariance[0] = -1.0;
        
        // Set covariance values
        imu_msg.linear_acceleration_covariance[0] = 0.01;
        imu_msg.linear_acceleration_covariance[4] = 0.01;
        imu_msg.linear_acceleration_covariance[8] = 0.01;
        
        imu_msg.angular_velocity_covariance[0] = 0.001;
        imu_msg.angular_velocity_covariance[4] = 0.001;
        imu_msg.angular_velocity_covariance[8] = 0.001;
        
        imu_pub_->publish(imu_msg);
    }
    
    void publishColorImage(const rs2::video_frame& frame, const rclcpp::Time& timestamp)
    {
        cv::Mat color_mat(cv::Size(frame.get_width(), frame.get_height()),
                         CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_mat).toImageMsg();
        msg->header.stamp = timestamp;
        msg->header.frame_id = "camera_color_optical_frame";
        
        color_pub_->publish(*msg);
    }
    
    void publishDepthImage(const rs2::depth_frame& frame, const rclcpp::Time& timestamp)
    {
        cv::Mat depth_mat(cv::Size(frame.get_width(), frame.get_height()),
                         CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        
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
    
    rs2::pipeline pipe_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    std::mutex imu_mutex_;
    std::shared_ptr<rs2::motion_frame> latest_accel_;
    std::shared_ptr<rs2::motion_frame> latest_gyro_;
    
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