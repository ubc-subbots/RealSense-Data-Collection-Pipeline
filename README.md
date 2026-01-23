# RealSense Data Collection Pipeline

ROS 2 pipeline for Intel RealSense D455 depth camera with real-time visualization and automated dataset collection. Built for UBC Subbots autonomous underwater vehicle.

## Architecture

**Publisher** (`realsense_publisher.cpp`) - C++ node using RealSense SDK for low-latency streaming  
**Subscriber** (`realsense_subscriber.py`) - Real-time visualization with distance analysis  
**Collector** (`realsense_data_collector.py`) - Automated dataset capture with motion detection

## Key Features

- Synchronized color and depth streaming at 30 FPS
- Real-time closest object detection and tracking
- Time-based and motion-triggered data collection
- Automatic metadata generation with depth statistics
- Direct SDK integration (no ROS wrapper dependency)

## Quick Start

```bash
# Terminal 1: Start camera publisher
ros2 run realsense_pipeline realsense_publisher

# Terminal 2: Visualize with distance overlay
ros2 run realsense_pipeline realsense_subscriber

# Terminal 3: Collect dataset (time-based)
ros2 run realsense_pipeline realsense_data_collector --mode time_based
```

## Dependencies

```bash
# RealSense SDK
sudo apt install librealsense2-dev librealsense2-utils

# ROS 2 Foxy + Python packages
sudo apt install ros-foxy-cv-bridge
pip3 install opencv-python numpy
```

## Data Collection

**Time-based mode** - Captures at regular intervals (default: 2s)
```bash
ros2 run realsense_pipeline realsense_data_collector --mode time_based --output dataset
```

**Motion-based mode** - Triggers on frame differences above threshold
```bash
ros2 run realsense_pipeline realsense_data_collector --mode motion --output dataset
```

**Output structure:**
```
dataset/
└── session_20240122_143052/
    ├── images/img_*.jpg
    ├── depth/depth_*.png
    └── metadata.json
```

Controls: `s` = manual capture, `q` = quit and save

## Visualization Features

- Green crosshair: center point distance
- Red crosshair: closest object in frame
- Real-time depth statistics (min/max/mean)
- Colormap depth rendering

## ROS Topics

| Topic | Type | Rate |
|-------|------|------|
| `/camera/color/image_raw` | sensor_msgs/Image (BGR8) | 30 Hz |
| `/camera/depth/image_raw` | sensor_msgs/Image (16UC1) | 30 Hz |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | 30 Hz |

## Performance

- Resolution: 640x480 (configurable to 1280x720)
- Latency: <50ms end-to-end
- CPU usage: ~15% single core (publisher)
- Depth range: 0.3m - 5m

## Parameters

**Publisher:**
```bash
ros2 run realsense_pipeline realsense_publisher --ros-args \
  -p width:=640 -p height:=480 -p fps:=30
```

**Collector:**
```bash
-p capture_interval:=2.0    # seconds between captures
-p motion_threshold:=5000   # pixel difference threshold
```

## Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/EraOfCoding/RealSense-Data-Collection-Pipeline.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Technical Details

- C++ publisher uses polling instead of callbacks for consistent frame timing
- Motion detection via OpenCV frame differencing with configurable threshold
- Metadata includes per-frame depth statistics for quality validation
- 16-bit depth stored as PNG for lossless compression

## Use Case

Developed for underwater object detection training data collection on UBC Subbots AUV. Pipeline enables systematic dataset generation with automated quality metrics and synchronized RGB-D capture.

---

**Stack:** ROS 2 Foxy | C++17 | Python 3.8 | OpenCV 4 | RealSense SDK 2.0