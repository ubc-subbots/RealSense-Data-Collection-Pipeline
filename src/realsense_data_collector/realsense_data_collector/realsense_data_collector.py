#!/usr/bin/env python3
"""
ROS2-based Underwater Dataset Collection
Subscribes to existing RealSense publisher
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from pathlib import Path
from datetime import datetime
import time


class ROS2DatasetCollector(Node):
    def __init__(self, output_dir='underwater_dataset', mode='time_based'):
        super().__init__('realsense_data_collector')
        
        self.output_dir = Path(output_dir)
        self.mode = mode
        self.bridge = CvBridge()
        
        # Create session directory
        self.session_name = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.session_dir = self.output_dir / self.session_name
        self.images_dir = self.session_dir / 'images'
        self.depth_dir = self.session_dir / 'depth'
        
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.depth_dir.mkdir(parents=True, exist_ok=True)
        
        # Settings
        self.declare_parameter('capture_interval', 2.0)
        self.declare_parameter('motion_threshold', 5000)
        self.capture_interval = self.get_parameter('capture_interval').value
        self.motion_threshold = self.get_parameter('motion_threshold').value
        
        # State
        self.image_counter = 0
        self.last_capture_time = 0
        self.previous_frame = None
        self.latest_color = None
        self.latest_depth = None
        
        # Metadata
        self.metadata = {
            'session_name': self.session_name,
            'start_time': datetime.now().isoformat(),
            'mode': mode,
            'images': []
        }
        
        # Subscribers
        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.color_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw',
            self.depth_callback, 10)
        
        # Timer for processing
        self.timer = self.create_timer(0.033, self.process_and_display)  # 30 Hz
        
        self.get_logger().info(f'Dataset Collector initialized: {self.session_name}')
        self.get_logger().info(f'Mode: {mode}, Interval: {self.capture_interval}s')
    
    def color_callback(self, msg):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Color conversion error: {e}')
    
    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def detect_motion(self, current_frame):
        if self.previous_frame is None:
            self.previous_frame = current_frame.copy()
            return False
        
        diff = cv2.absdiff(self.previous_frame, current_frame)
        gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray_diff, 25, 255, cv2.THRESH_BINARY)
        changed_pixels = cv2.countNonZero(thresh)
        
        self.previous_frame = current_frame.copy()
        return changed_pixels > self.motion_threshold
    
    def should_capture(self):
        current_time = time.time()
        
        if self.mode == 'time_based':
            if current_time - self.last_capture_time >= self.capture_interval:
                self.last_capture_time = current_time
                return True, "Time interval"
        
        elif self.mode == 'motion':
            if self.latest_color is not None and self.detect_motion(self.latest_color):
                self.last_capture_time = current_time
                return True, "Motion detected"
        
        return False, ""
    
    def save_frame(self, color_image, depth_image, trigger_reason):
        self.image_counter += 1
        
        img_name = f"img_{self.image_counter:06d}.jpg"
        depth_name = f"depth_{self.image_counter:06d}.png"
        
        img_path = self.images_dir / img_name
        depth_path = self.depth_dir / depth_name
        
        cv2.imwrite(str(img_path), color_image)
        cv2.imwrite(str(depth_path), depth_image)
        
        valid_depth = depth_image[depth_image > 0]
        depth_stats = {
            'min': int(np.min(valid_depth)) if len(valid_depth) > 0 else 0,
            'max': int(np.max(valid_depth)) if len(valid_depth) > 0 else 0,
            'mean': float(np.mean(valid_depth)) if len(valid_depth) > 0 else 0.0
        }
        
        image_metadata = {
            'id': self.image_counter,
            'filename': img_name,
            'depth_filename': depth_name,
            'timestamp': datetime.now().isoformat(),
            'trigger': trigger_reason,
            'depth_stats': depth_stats
        }
        self.metadata['images'].append(image_metadata)
        
        self.get_logger().info(
            f"Saved {img_name} ({trigger_reason}) - Depth: {depth_stats['mean']:.0f}mm"
        )
    
    def process_and_display(self):
        if self.latest_color is None or self.latest_depth is None:
            return
        
        # Check if should capture
        should_save, reason = self.should_capture()
        if should_save:
            self.save_frame(self.latest_color, self.latest_depth, reason)
        
        # Create display
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(self.latest_depth, alpha=0.03),
            cv2.COLORMAP_JET
        )
        
        display = np.hstack([self.latest_color, depth_colormap])
        
        info = [
            f"Session: {self.session_name}",
            f"Mode: {self.mode}",
            f"Captured: {self.image_counter}",
            f"Press 'q' to quit"
        ]
        
        y = 30
        for text in info:
            cv2.putText(display, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                       0.6, (255, 255, 255), 2)
            cv2.putText(display, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                       0.6, (0, 255, 0), 1)
            y += 30
        
        cv2.imshow('ROS2 Dataset Collector', display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.cleanup()
            rclpy.shutdown()
        elif key == ord('s'):
            self.save_frame(self.latest_color, self.latest_depth, "Manual")
    
    def cleanup(self):
        self.metadata['end_time'] = datetime.now().isoformat()
        self.metadata['total_images'] = self.image_counter
        
        metadata_path = self.session_dir / 'metadata.json'
        with open(metadata_path, 'w') as f:
            json.dump(self.metadata, f, indent=2)
        
        self.get_logger().info(f'Collection complete: {self.image_counter} images')
        self.get_logger().info(f'Saved to: {self.session_dir}')
        
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', '-o', default='underwater_dataset')
    parser.add_argument('--mode', '-m', choices=['time_based', 'motion'], 
                       default='time_based')
    
    args = parser.parse_args()
    
    node = ROS2DatasetCollector(output_dir=args.output, mode=args.mode)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()