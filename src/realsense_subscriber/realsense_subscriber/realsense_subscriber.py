#!/usr/bin/env python3
"""
RealSense Subscriber with YOLO Object Detection

This node subscribes to RealSense camera topics, runs YOLO detection,
and displays bounding boxes with distance measurements.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from datetime import date, datetime
import json


class RealSenseYoloSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_yolo_subscriber')
        
        self.bridge = CvBridge()

        
        self.declare_parameter('model_path', '/home/eraofcoding/Subbots/deepsense_test/src/realsense_subscriber/realsense_subscriber/best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('show_color', True)
        self.declare_parameter('show_depth', True)
        self.declare_parameter('save_images', False) # Toggle for automatic saving
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('save_dir', '/home/eraofcoding/Subbots/deepsense_test/dataset_collection')
        
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.show_color = self.get_parameter('show_color').value
        self.show_depth = self.get_parameter('show_depth').value
        self.save_images = self.get_parameter('save_images').value
        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value

        self.save_next_frame = False
        
        self.save_dir = self.get_parameter('save_dir').value
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.get_logger().info('='*60)
        self.get_logger().info('RealSense YOLO Subscriber')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        
        try:
            self.model = YOLO(model_path)
            self.model.to(device)
            self.get_logger().info(f'✓ YOLO model loaded successfully on {device}')
        except Exception as e:
            self.get_logger().error(f'✗ Failed to load YOLO model: {str(e)}')
            raise
        
        self.image_count = 0
        
        # Latest images
        self.latest_color = None
        self.latest_depth = None
        
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
        
        self.get_logger().info(f'Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'Show Color: {self.show_color}, Show Depth: {self.show_depth}')
        self.get_logger().info('='*60)
        
        self.camera_info = None
        
    def color_callback(self, msg):
        """Callback for color image messages"""
        try:
            # Convert ROS Image message to OpenCV format
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
        except Exception as e:
            self.get_logger().error(f'Error processing color image: {str(e)}')
    
    def depth_callback(self, msg):
        """Callback for depth image messages"""
        try:
            # Convert ROS Image message to OpenCV format (16-bit depth)
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            
            if self.latest_color is not None and self.latest_depth is not None:
                self.process_and_display()
            
            if self.save_images and self.save_next_frame == True and self.latest_color is not None:
                os.makedirs(self.save_dir, exist_ok=True)
                dated_dir = f'{self.save_dir}/{date.today()}'
                os.makedirs(dated_dir, exist_ok=True)
                filename = f'{dated_dir}/detection_{self.image_count}.jpg'
                cv2.imwrite(filename, self.latest_color)
                self.save_metadata(dated_dir, filename)
                self.get_logger().info(f'Saved {dated_dir}/detection_{self.image_count}.jpg')
                self.save_next_frame = False
                
            self.image_count += 1
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def save_metadata(self, dated_dir, filename):
        metadata_path = f'{dated_dir}/metadata.json'
        
        if os.path.exists(metadata_path):
            with open(metadata_path, 'r') as f:
                metadata = json.load(f)
        else:
            metadata = {'captures': []}
        
        detections = []
        if self.latest_color is not None:
            results = self.model(self.latest_color, conf=self.conf_threshold, verbose=False)
            if len(results) > 0:
                for box in results[0].boxes:
                    class_id = int(box.cls.cpu().numpy())
                    detections.append({
                        'class': results[0].names[class_id],
                        'confidence': round(float(box.conf.cpu().numpy()), 4),
                        'bbox': list(map(int, box.xyxy.cpu().numpy()[0])),
                    })
        
        center_y, center_x = self.latest_depth.shape[0] // 2, self.latest_depth.shape[1] // 2
        center_depth = self.latest_depth[center_y, center_x]
        
        entry = {
            'filename': filename,
            'timestamp': datetime.now().isoformat(),
            'frame_number': self.image_count,
            'resolution': {
                'width': self.latest_color.shape[1],
                'height': self.latest_color.shape[0],
            },
            'center_distance_m': round(center_depth / 1000.0, 4) if center_depth > 0 else None,
            'confidence_threshold': self.conf_threshold,
            'detections': detections,
        }
        
        metadata['captures'].append(entry)
        
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        self.get_logger().info(f'Updated metadata.json with {len(detections)} detection(s)')
    
    def camera_info_callback(self, msg):
        """Callback for camera info messages"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera Info received:')
            self.get_logger().info(f'  Resolution: {msg.width}x{msg.height}')
            self.get_logger().info(f'  Focal Length: fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}')
            self.get_logger().info(f'  Principal Point: cx={msg.k[2]:.2f}, cy={msg.k[5]:.2f}')
    
    def process_and_display(self):
        """Process depth and color images with YOLO detection"""
        color_display = self.latest_color.copy()
        depth_image = self.latest_depth.copy()
        
        height, width = depth_image.shape
        center_x, center_y = width // 2, height // 2
        
        results = self.model(self.latest_color, conf=self.conf_threshold, verbose=False)

        box_color = (0, 255, 0) # Green
        pointer_color = (128, 0, 128) # Purple
        
        if len(results) > 0:
            result = results[0]
            boxes = result.boxes
            
            if boxes is not None and len(boxes) > 0:
                detection_info = []
                
                for box in boxes:
                    xyxy = box.xyxy.cpu().numpy()[0]
                    x1, y1, x2, y2 = map(int, xyxy)
                    
                    class_id = int(box.cls.cpu().numpy())
                    class_name = result.names[class_id]
                    confidence = float(box.conf.cpu().numpy())
                    
                    bbox_center_x = int((x1 + x2) / 2)
                    bbox_center_y = int((y1 + y2) / 2)
                    
                    distance_str = ""
                    if 0 <= bbox_center_x < width and 0 <= bbox_center_y < height:
                        depth_value = depth_image[bbox_center_y, bbox_center_x]
                        if depth_value > 0:
                            distance_m = depth_value / 1000.0
                            distance_str = f" @ {distance_m:.2f}m"
                    
                    cv2.rectangle(color_display, (x1, y1), (x2, y2), box_color, 2)
                    
                    cv2.circle(color_display, (bbox_center_x, bbox_center_y), 5, (0, 0, 255), -1)
                    
                    label = f"{class_name}: {confidence:.2f}{distance_str}"
                    
                    (label_width, label_height), baseline = cv2.getTextSize(
                        label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
                    )
                    cv2.rectangle(
                        color_display,
                        (x1, y1 - label_height - 10),
                        (x1 + label_width, y1),
                        box_color,
                        -1
                    )
                    
                    cv2.putText(
                        color_display,
                        label,
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 0),
                        2
                    )
                    
                    detection_info.append(f"{class_name}:{confidence:.2f}{distance_str}")
                
                if detection_info:
                    self.get_logger().info(
                        f'Detected: {", ".join(detection_info)}',
                        throttle_duration_sec=1.0
                    )
        
        center_distance = depth_image[center_y, center_x]
        
        cv2.line(color_display, (center_x - 30, center_y), 
                (center_x + 30, center_y), pointer_color, 2)
        cv2.line(color_display, (center_x, center_y - 30), 
                (center_x, center_y + 30), pointer_color, 2)
        cv2.circle(color_display, (center_x, center_y), 5, pointer_color, 2)
        
        if center_distance > 0:
            distance_m = center_distance / 1000.0
            text = f"Center: {distance_m:.2f}m"
            cv2.putText(color_display, text, (center_x + 40, center_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, pointer_color, 2)
        
        valid_depths = depth_image[depth_image > 0]
        if len(valid_depths) > 0:
            min_dist = np.min(valid_depths)
            max_dist = np.max(valid_depths)
            avg_dist = np.mean(valid_depths)
            
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
        
        if self.show_color:
            cv2.imshow('YOLO Detection with Distance', color_display)
        
        if self.show_depth:
            # Normalize depth for visualization (0-5000mm to 0-255)
            depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)
            mask = depth_image > 0
            depth_normalized[mask] = np.clip(depth_image[mask] / 5000.0 * 255, 0, 255).astype(np.uint8)
            
            # Apply colormap
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # Mark center on depth map
            cv2.circle(depth_colormap, (center_x, center_y), 5, (255, 255, 255), 2)
            
            cv2.imshow('Depth Image', depth_colormap)
        
        # Wait for key press (1ms)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('s'):
            self.get_logger().info('Saving next frame')
            self.save_next_frame = True
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        subscriber = RealSenseYoloSubscriber()
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        print('\nShutdown requested by user')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()