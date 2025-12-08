# Ouster LiDAR

## Image
<img width="20%" src="https://cdn.prod.website-files.com/64d23a301670476693286174/65511136864d515f87336dd6_ouster_os1_d9caed355c.png">



## File
```
/sensors_drivers/ouster-ros/param/params.yaml
```


## For IMU Driver 
*See [Xsens IMU](#-xsens-imu) section below for setup instructions*


## Changes 
```yaml
ML:
  XsensMtiNode:
    port: /dev/imu_xsens   # Set Udev rule
    publisher_queue_size: 5
    pub_imu: true  #For getting xsens-imu data set it true


    Ouster:
      OsNode:
        sensor_hostname: os-122403001858.local # Enter ouster LiDAR IP 
        replay: true
        metadata: /home/user/.ros/os-122403001858.local.json # Change user and LiDAR IP
        udp_dest: 127.0.0.1 #Set your Host IP
```

## Launch File

```bash
roslaunch ouster-ros os.launch
```

## Note:
 `os.launch` is a single self-contained file that sets up everything the Ouster sensor needs to run.



## Key Parameters to Set
- `sensor_hostname`: IP address of the Ouster LiDAR sensor
- `udp_dest`: IP address of the host computer receiving data
- `lidar_port`: UDP port for lidar data (default: 7502)
- `imu_port`: UDP port for IMU data (default: 7503)
- `lidar_mode`: Resolution and rate, e.g., 512x10, 512x20, 1024x10, 1024x20, 2048x10, 4096x5
- `timestamp_mode`: TIME_FROM_ROS_TIME, TIME_FROM_INTERNAL_OSC, TIME_FROM_SYNC_PULSE_IN, TIME_FROM_PTP_1588
- `viz`: Set to true to launch RViz visualization (default: true)

## Optional Parameters
- `tf_prefix`: Namespace for TF transforms
- `sensor_frame`: TF frame name for sensor (default: os_sensor)
- `lidar_frame`: TF frame name for lidar (default: os_lidar)
- `imu_frame`: TF frame name for IMU (default: os_imu)
- `proc_mask`: Processing mask (default: IMG|PCL|IMU|SCAN|TLM)
- `organized`: Generate organized point cloud (default: true)
- `destagger`: Enable point cloud destaggering (default: true)


---