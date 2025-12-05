# 🔷 Livox LiDAR
<img width="20%" src="https://www.livoxtech.com/dps/2d9e037e6d457ef7ffec037f7d16dcf8.png" />

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
      "log_data_ip" : "192.168.X.XX",  //Enter Host IP Here
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.X.XX", //Enter LiDAR IP Here
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {  //Add TF information (radians for rotation, meters for translation)
        "roll": 0.0,  // radians
        "pitch": 0.0, // radians
        "yaw": 0.0,   // radians
        "x": 0,       // meters
        "y": 0,       // meters
        "z": 0        // meters
      }
    }
  ]
```

>**Note:**
 For multiple Livox LiDARs, copy and paste the `MID360` and `lidar_configs` sections, then change the IP address of each LiDAR according to your setup.

**File:**
```
/sensors_drivers/livox_ros_driver2/launch_ROS1/msg_MID360.launch
```

 ```xml
 <!-- For More Than 1 LiDAR -->
 <arg name="multi_topic" default="0"/> <!-- Change to 1 for multiple LiDARs -->
 ```

 ---