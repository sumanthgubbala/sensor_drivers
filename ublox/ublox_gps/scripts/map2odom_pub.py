#!/usr/bin/env python

import rospy
import yaml
import tf2_ros
import geometry_msgs.msg
import math
import os

# 🔧 Hardcoded YAML path (set yours here)
yaml_path = "/home/fluxauto/ros_ws/src/my_package/params/config.yaml"

def load_yaw_from_yaml(yaml_path):
    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)
        return data.get("map2odomrad", 0.0)

def publish_static_tf(yaw):
    quat = tf2_ros.transformations.quaternion_from_euler(0, 0, yaw)

    static_transform_stamped = geometry_msgs.msg.TransformStamped()
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "map"
    static_transform_stamped.child_frame_id = "odom"

    static_transform_stamped.transform.translation.x = 0.0
    static_transform_stamped.transform.translation.y = 0.0
    static_transform_stamped.transform.translation.z = 0.0

    static_transform_stamped.transform.rotation.x = quat[0]
    static_transform_stamped.transform.rotation.y = quat[1]
    static_transform_stamped.transform.rotation.z = quat[2]
    static_transform_stamped.transform.rotation.w = quat[3]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(static_transform_stamped)

    rospy.loginfo("Published static transform with yaw offset: %.3f rad", yaw)

if __name__ == '__main__':
    rospy.init_node('map_to_odom_tf_publisher')

    if not os.path.isfile(yaml_path):
        rospy.logerr("YAML file not found: %s", yaml_path)
        exit(1)

    yaw_offset = load_yaw_from_yaml(yaml_path)
    publish_static_tf(yaw_offset)

    rospy.spin()

