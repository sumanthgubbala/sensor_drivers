
# SICK 2D LiDAR

## Image

<img width="20%" src="https://cdn.sick.com/media/pim/5/15/215/IM0085215.png"/>

## File

```
/sensors_drivers/sick_safetyscanners/launch/sick_safetyscanners.launch
```

## Set

```xml
    <arg name="Right_Sensor_IP"  default="169.254.X.XXX" />  <!-- Enter SICK LiDAR IP -->
    <arg name="Left_Sensor_IP"  default="169.254.X.XXX" />  <!-- Enter SICK LiDAR IP -->

    <arg name="host_ip_right"  default="169.254.X.XXX" />  <!-- Enter Host IP for Right Sensor -->
    <arg name="host_ip_left"  default="169.254.X.XXX" />  <!-- Enter Host IP for Left Sensor -->


    <!-- For TF transforms, uncomment the below 3 lines in sick_safetyscanners.launch file -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_laser_right" args="-0.30 -0.52 0 -1.525 0.0  3.141583   base_link laser_right" />
    <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_laser_left" args="0.08  0.49 0 1.5708 0.0 3.141583  base_link laser_left" />
    <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_lidar_center" args="0.28 0.0 0.0 3.141583 0.0 0.0 base_link laser_center"/> 

```

---