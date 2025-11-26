#!/usr/bin/env python3

import rospy
import yaml
import tf2_ros
import geometry_msgs.msg
import os
from tf.transformations import quaternion_from_euler

def load_yaw_from_config(config_path):
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            if config is None:
                rospy.logwarn("Config file is empty. Defaulting yaw to 0.0")
                return 0.0
            yaw = config.get('map2odomRad', None)
            if yaw is None:
                rospy.logwarn("'map2odomRad' not found in config. Defaulting yaw to 0.0")
                return 0.0
            return float(yaw)
    except Exception as e:
        rospy.logerr(f"Failed to load config: {e}")
        return 0.0

def publish_static_tf(yaw):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transform_stamped = geometry_msgs.msg.TransformStamped()

    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "map"
    static_transform_stamped.child_frame_id = "odom"

    static_transform_stamped.transform.translation.x = 0.0
    static_transform_stamped.transform.translation.y = 0.0
    static_transform_stamped.transform.translation.z = 0.0

    quat = quaternion_from_euler(0.0, 0.0, yaw)  # roll, pitch, yaw
    static_transform_stamped.transform.rotation.x = quat[0]
    static_transform_stamped.transform.rotation.y = quat[1]
    static_transform_stamped.transform.rotation.z = quat[2]
    static_transform_stamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transform_stamped)
    rospy.loginfo(f"Published static transform map -> odom with yaw: {yaw:.4f} rad")

if __name__ == '__main__':
    rospy.init_node('map_to_odom_tf_publisher')

    config_path = "/home/fluxauto/ua/ceres_dep_dev_ws/src/ceres_pp_stack/ceres_pp/planner_core/config/ab.yaml"
    yaw = load_yaw_from_config(config_path)

    publish_static_tf(yaw)

    rospy.spin()  # Keep the node alive
