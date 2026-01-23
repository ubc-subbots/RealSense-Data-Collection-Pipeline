#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class RealSenseSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_subscriber')
        
        # Create CV Bridge for converting ROS Image messages to OpenCV format
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('show_color', True)
        self.declare_parameter('show_depth', True)
        self.declare_parameter('save_images', False)
        self.declare_parameter('show_closest_object', True)
        self.declare_parameter('show_center_distance', True)
        
        # Get parameters
        self.show_color = self.get_parameter('show_color').value
        self.show_depth = self.get_parameter('show_depth').value
        self.save_images = self.get_parameter('save_images').value
        self.show_closest_object = self.get_parameter('show_closest_object').value
        self.show_center_distance = self.get_parameter('show_center_distance').value
        
        # Image counter for saving
        self.image_count = 0
        
        # Store latest images for synchronized processing
        self.latest_color = None
        self.latest_depth = None
        
        # Create subscribers
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.get_logger().info('RealSense Subscriber initialized')
        self.get_logger().info(f'Show Color: {self.show_color}, Show Depth: {self.show_depth}')
        self.get_logger().info(f'Show Closest Object: {self.show_closest_object}')
        
        # Store camera info
        self.camera_info = None
        
    def color_callback(self, msg):
        """
        Callback for color image messages
        """
        try:
            # Convert ROS Image message to OpenCV format
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Save image if enabled
            if self.save_images:
                filename = f'color_image_{self.image_count}.jpg'
                cv2.imwrite(filename, self.latest_color)
                self.get_logger().info(f'Saved {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing color image: {str(e)}')
    
    def depth_callback(self, msg):
        """
        Callback for depth image messages
        """
        try:
            # Convert ROS Image message to OpenCV format (16-bit depth)
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            
            # Process and display combined visualization
            if self.latest_color is not None and self.latest_depth is not None:
                self.process_and_display()
            
            # Save depth image if enabled
            if self.save_images:
                filename = f'depth_image_{self.image_count}.png'
                cv2.imwrite(filename, self.latest_depth)
                self.get_logger().info(f'Saved {filename}')
                self.image_count += 1
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def camera_info_callback(self, msg):
        """
        Callback for camera info messages
        """
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera Info received:')
            self.get_logger().info(f'  Resolution: {msg.width}x{msg.height}')
            self.get_logger().info(f'  Focal Length: fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}')
            self.get_logger().info(f'  Principal Point: cx={msg.k[2]:.2f}, cy={msg.k[5]:.2f}')
    
    def process_and_display(self):
        """
        Process depth and color images together with distance information
        """
        color_display = self.latest_color.copy()
        depth_image = self.latest_depth.copy()
        
        # Get image dimensions
        height, width = depth_image.shape
        center_x, center_y = width // 2, height // 2
        
        # === CLOSEST OBJECT DETECTION ===
        if self.show_closest_object:
            # Find closest object (minimum non-zero depth)
            valid_depth_mask = depth_image > 0
            if np.any(valid_depth_mask):
                min_distance = np.min(depth_image[valid_depth_mask])
                
                # Find position(s) of closest object
                closest_positions = np.where(depth_image == min_distance)
                
                if len(closest_positions[0]) > 0:
                    # Get the first closest point (or you could use centroid)
                    closest_y = closest_positions[0][0]
                    closest_x = closest_positions[1][0]
                    
                    # Draw marker on color image
                    cv2.circle(color_display, (closest_x, closest_y), 10, (0, 0, 255), 2)
                    cv2.circle(color_display, (closest_x, closest_y), 3, (0, 0, 255), -1)
                    
                    # Draw crosshair
                    cv2.line(color_display, (closest_x - 20, closest_y), 
                            (closest_x + 20, closest_y), (0, 0, 255), 2)
                    cv2.line(color_display, (closest_x, closest_y - 20), 
                            (closest_x, closest_y + 20), (0, 0, 255), 2)
                    
                    # Display distance text
                    distance_m = min_distance / 1000.0
                    text = f"CLOSEST: {distance_m:.2f}m"
                    cv2.putText(color_display, text, (closest_x + 25, closest_y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    # Log to console
                    self.get_logger().info(
                        f'Closest object at ({closest_x}, {closest_y}): {min_distance}mm ({distance_m:.2f}m)',
                        throttle_duration_sec=1.0  # Log once per second
                    )
        
        # === CENTER DISTANCE ===
        if self.show_center_distance:
            # Get distance at center of image
            center_distance = depth_image[center_y, center_x]
            
            # Draw center crosshair
            cv2.line(color_display, (center_x - 30, center_y), 
                    (center_x + 30, center_y), (0, 255, 0), 2)
            cv2.line(color_display, (center_x, center_y - 30), 
                    (center_x, center_y + 30), (0, 255, 0), 2)
            cv2.circle(color_display, (center_x, center_y), 5, (0, 255, 0), 2)
            
            # Display distance text
            if center_distance > 0:
                distance_m = center_distance / 1000.0
                text = f"Center: {distance_m:.2f}m"
                cv2.putText(color_display, text, (center_x + 40, center_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                text = "Center: No data"
                cv2.putText(color_display, text, (center_x + 40, center_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # === DEPTH STATISTICS ===
        valid_depths = depth_image[depth_image > 0]
        if len(valid_depths) > 0:
            min_dist = np.min(valid_depths)
            max_dist = np.max(valid_depths)
            avg_dist = np.mean(valid_depths)
            
            # Display statistics on image
            stats_text = [
                f"Min: {min_dist}mm ({min_dist/1000:.2f}m)",
                f"Max: {max_dist}mm ({max_dist/1000:.2f}m)",
                f"Avg: {avg_dist:.0f}mm ({avg_dist/1000:.2f}m)"
            ]
            
            y_offset = 30
            for i, text in enumerate(stats_text):
                cv2.putText(color_display, text, (10, y_offset + i*30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(color_display, text, (10, y_offset + i*30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
        # === DISPLAY IMAGES ===
        if self.show_color:
            cv2.imshow('Color Image with Distance Info', color_display)
        
        if self.show_depth:
            # Normalize depth for visualization (0-5000mm to 0-255)
            depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)
            mask = depth_image > 0
            depth_normalized[mask] = np.clip(depth_image[mask] / 5000.0 * 255, 0, 255).astype(np.uint8)
            
            # Apply colormap for better visualization
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # Mark closest object on depth map too
            if self.show_closest_object and np.any(valid_depth_mask):
                cv2.circle(depth_colormap, (closest_x, closest_y), 10, (255, 255, 255), 2)
            
            # Mark center on depth map
            if self.show_center_distance:
                cv2.circle(depth_colormap, (center_x, center_y), 5, (255, 255, 255), 2)
            
            cv2.imshow('Depth Image', depth_colormap)
        
        cv2.waitKey(1)
    
    def __del__(self):
        """
        Cleanup when node is destroyed
        """
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        subscriber = RealSenseSubscriber()
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()