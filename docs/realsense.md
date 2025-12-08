
# RealSense Cameras

## Image

<img width="20%" src="https://d2t1xqejof9utc.cloudfront.net/screenshots/pics/3c5d6807aee4bccf8a693e04725f15f4/large.png">


## Configuration and Setup

### File
```
/sensors_drivers/realsense-ros/realsense2_camera/launch/rs_camera.launch
```


## Key Parameters to Change
- `serial_no`: Camera serial number (required for multi-camera setups)
- `camera`: Namespace for camera topics (`default: camera`)
- `enable_pointcloud`: Generate point cloud data (`set to true for navigation`)
- `enable_depth`: Enable depth stream (`default: true`)
- `enable_color`: Enable RGB color stream (`default: true`)
- `enable_infra1/enable_infra2`: Enable infrared streams for stereo vision
- `depth_width/depth_height`: Depth resolution (`default: -1 for auto`)
- `color_width/color_height`: RGB resolution (`default: -1 for auto`)
- `depth_fps/color_fps`: Frame rates (`30/60` fps typical)
- `enable_gyro/enable_accel`: Enable IMU data (set to true for odometry)
- `publish_tf`: Publish camera TF transforms (`default: true`)
- `tf_prefix`: Prefix for TF frames (`default: realsense`)
- `enable_sync`: Enable camera synchronization (`default: false`)
- `align_depth`: Align depth frames to color camera (`default: false`)

## Resolution Options
- 640x480 (VGA)
- 848x480 (Wide VGA)
- 1280x720 (HD)
- 1920x1080 (Full HD)

## Available Launch Files
- `rs_camera.launch`: Basic camera launch
- `rs_d435_camera_with_model.launch`: D435 with URDF model and RViz
- `rs_t265.launch`: T265 tracking camera only
- `rs_d400_and_t265.launch`: Combined D400 + T265 setup
- `rs_multiple_devices.launch`: Multi-camera setup
- `rs_aligned_depth.launch`: Depth aligned to color frame
- `demo_pointcloud.launch`: Point cloud visualization demo
