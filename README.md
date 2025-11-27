# 📄 Sensor Drivers for Autonomous Vehicle (ROS Noetic)

This folder contains all sensor drivers for an autonomous vehicle running **ROS Noetic (Ubuntu 20.04)**.

Each driver includes:

* `launch/`
* `config/`
* `src/` *(if custom modifications are needed)*

---

## 🔷 Livox LiDAR

<p style="flux: left;"><img width="20%" src="https://www.livoxtech.com/dps/2d9e037e6d457ef7ffec037f7d16dcf8.png" /></p>

**File:**

```
/sensors_drivers/livox_ros_driver2/config/MID360_config.json
```

**Change:**

```json
  "MID360": {
    "host_net_info" : {
      "cmd_data_ip" : "192.168.X.XX", //Enter Host IP Here
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.X.XX", //Enter Host IP Here
      "push_msg_port": 56201,
      "point_data_ip": "192.168.x.xx", //Enter Host IP Here
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.X.XX", //Enter Host IP Here
      "imu_data_port": 56401,
      "log_data_ip" : "92.168.X.XX",  //Enter Host IP Here
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.X.XX", //Enter LiDAR IP Here
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {  //Add TF information
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
```


---

## 🔷 SICK 2D LiDAR

<img width="20%" src="https://cdn.sick.com/media/pim/5/15/215/IM0085215.png"/>

**File:**

```
/sensors_drivers/sick_safetyscanners/launch/sick_safetyscanners.launch
```

**Set:**

```xml
    <arg name="Right_Sensor_IP"  default="169.254.X.XXX" />  <!-- Enter SICK LiDAR IP -->
    <arg name="Left_Sensor_IP"  default="169.254.X.XXX" />  <!-- Enter SICK LiDAR IP -->

    <arg name="host_ip_right"  default="169.254.X.XXX" />  <!-- Enter Host IP for Left/Right -->
```

---

## 🔷 Ouster LiDAR

<img width="20%" src="https://cdn.prod.website-files.com/64d23a301670476693286174/65511136864d515f87336dd6_ouster_os1_d9caed355c.png">



**File:**
```
sensors_drivers/ouster-ros/launch/sensor.launch
```


**Key Parameters to Set:**
- `sensor_hostname`: IP address of the Ouster LiDAR sensor
- `udp_dest`: IP address of the host computer receiving data
- `lidar_port`: UDP port for lidar data (default: 7502)
- `imu_port`: UDP port for IMU data (default: 7503)
- `lidar_mode`: Resolution and rate, e.g., 512x10, 512x20, 1024x10, 1024x20, 2048x10, 4096x5
- `timestamp_mode`: TIME_FROM_ROS_TIME, TIME_FROM_INTERNAL_OSC, TIME_FROM_SYNC_PULSE_IN, TIME_FROM_PTP_1588
- `viz`: Set to true to launch RViz visualization (default: true)

**Optional Parameters:**
- `tf_prefix`: Namespace for TF transforms
- `sensor_frame`: TF frame name for sensor (default: os_sensor)
- `lidar_frame`: TF frame name for lidar (default: os_lidar)
- `imu_frame`: TF frame name for IMU (default: os_imu)
- `proc_mask`: Processing mask (default: IMG|PCL|IMU|SCAN|TLM)
- `organized`: Generate organized point cloud (default: true)
- `destagger`: Enable point cloud destaggering (default: true)


---

## 2. How to Change Serial Device (GPS / IMU)

### 🔷 GPS (u-blox F9P)

<img width="20%" src="https://www.gateworks.com/wp-content/uploads/gw16143-angle-800px.png">

**File:**

```
/sensors_drivers/ublox/ublox_gps/config/ardusimple_2.yaml
```

**Change:**

```yaml
device: /dev/ttyACM1  # Set the Udev rule for RTK device and Enter the Udev address here
```

**File:**
```
sensors_drivers/ublox/ublox_gps/launch/ardusimple.launch
```


**Key Parameters:**
- `device`: Serial device path (set via Udev rule)
- `frame_id`: TF frame for GPS data (default: gps)
- `config_on_startup`: Whether to configure GPS on startup (default: false)
- `uart1/baudrate`: Serial baud rate (default: 460800)
- `rate`: Measurement rate in Hz
- `nav_rate`: Navigation rate in Hz
- `publish/all`: Publish all available GPS data (default: true)

**Available Launch Files:**
- `ardusimple.launch`: Basic GPS launch with configuration
- `ekf.launch`: GPS with EKF fusion
- `mapping.launch`: GPS for mapping applications
- `ublox_device.launch`: Generic ublox device launch






---

### 🔷 Xsens IMU

<img width="20%" src="https://www.mouser.in/images/marketingid/2023/img/109066689.png?v=061025.0729">

**NOTE :**

```
 No Need to Make Any Changes 
```


---


### 🔷 RealSense Cameras

<img width="20%" src="https://d2t1xqejof9utc.cloudfront.net/screenshots/pics/3c5d6807aee4bccf8a693e04725f15f4/large.png">

**File:**

```
/sensors_drivers/realsense-ros/realsense2_camera/launch/rs_camera.launch
```


**Key Parameters to Change:**
- `serial_no`: Camera serial number (required for multi-camera setups)
- `camera`: Namespace for camera topics (default: camera)
- `enable_pointcloud`: Generate point cloud data (set to true for navigation)
- `enable_depth`: Enable depth stream (default: true)
- `enable_color`: Enable RGB color stream (default: true)
- `enable_infra1/enable_infra2`: Enable infrared streams for stereo vision
- `depth_width/depth_height`: Depth resolution (default: -1 for auto)
- `color_width/color_height`: RGB resolution (default: -1 for auto)
- `depth_fps/color_fps`: Frame rates (30/60 fps typical)
- `enable_gyro/enable_accel`: Enable IMU data (set to true for odometry)
- `publish_tf`: Publish camera TF transforms (default: true)
- `tf_prefix`: Prefix for TF frames (default: realsense)

**Resolution Options:**
- 640x480 (VGA)
- 848x480 (Wide VGA)
- 1280x720 (HD)
- 1920x1080 (Full HD)

**Available Launch Files:**
- `rs_camera.launch`: Basic camera launch
- `rs_d435_camera_with_model.launch`: D435 with URDF model and RViz
- `rs_t265.launch`: T265 tracking camera only
- `rs_d400_and_t265.launch`: Combined D400 + T265 setup
- `rs_multiple_devices.launch`: Multi-camera setup
- `rs_aligned_depth.launch`: Depth aligned to color frame
- `demo_pointcloud.launch`: Point cloud visualization demo





