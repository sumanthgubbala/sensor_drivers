#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import yaml
import os
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import euler_from_quaternion

YAML_PATH = os.path.expanduser('/home/fluxauto/ua/rtk_ws/src/ceres_localisation_ws/ublox/ublox_gps/config/dynamic_tf.yaml')
MIN_DIST = 5.0  # minimum distance for offset correction

class ManualHeadingOffsetCorrector:
    def __init__(self):
        rospy.init_node("heading_tf_adjuster")

        # State
        self.fix_session_active = False
        self.motion_start_pose = None
        self.motion_end_pose = None
        self.offset_applied = False

        # Load TF from YAML
        self.tf_data = self.load_tf_yaml()
        self.current_yaw_tf = self.tf_data.get('base_gps_yaw_tf', 0.0)

        # TF Broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Subscribers
        rospy.Subscriber("/ML/VehiclePose", PoseStamped, self.pose_callback)

        # Timer for param monitoring and TF publishing
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)  # 10 Hz

    def pose_callback(self, msg):
        pos = msg.pose.position
        curr_pos = (pos.x, pos.y)

        # GNSS heading
        q = msg.pose.orientation
        _, _, gnss_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        gnss_yaw = self.normalize_angle(gnss_yaw)

        if self.fix_session_active:
            if self.motion_start_pose is None:
                # Start point of motion
                self.motion_start_pose = curr_pos
                return

            # Check distance moved
            dx = curr_pos[0] - self.motion_start_pose[0]
            dy = curr_pos[1] - self.motion_start_pose[1]
            dist = math.hypot(dx, dy)

            if dist >= MIN_DIST:
                self.motion_end_pose = curr_pos
                self.motion_end_yaw = gnss_yaw
        else:
            # Do nothing when not in correction session
            return

    def timer_callback(self, event):
        # Always publish current TF
        self.publish_tf(self.current_yaw_tf)

        fix_param = rospy.get_param("/fix_heading_offset", False)

        if fix_param and not self.fix_session_active:
            # User initiated correction session
            rospy.loginfo("[TF Adjuster] Heading correction session started.")
            self.fix_session_active = True
            self.motion_start_pose = None
            self.motion_end_pose = None
            self.offset_applied = False

        elif not fix_param and self.fix_session_active and not self.offset_applied:
            # User ended correction session – time to apply offset
            if self.motion_start_pose and self.motion_end_pose:
                dx = self.motion_end_pose[0] - self.motion_start_pose[0]
                dy = self.motion_end_pose[1] - self.motion_start_pose[1]
                motion_heading = math.atan2(dy, dx)
                motion_heading = self.normalize_angle(motion_heading)

                # offset = self.normalize_angle(self.motion_end_yaw - motion_heading)
                # self.current_yaw_tf = self.normalize_angle(self.current_yaw_tf - offset)
                offset = self.normalize_angle(motion_heading - self.motion_end_yaw)
                self.current_yaw_tf = self.normalize_angle(self.current_yaw_tf - offset)

                # Update YAML
                self.tf_data['base_gps_yaw_tf'] = self.current_yaw_tf
                self.write_tf_yaml(self.tf_data)

                rospy.logwarn(f"[TF Adjuster] Applied heading offset: {offset:.4f} rad. New yaw: {self.current_yaw_tf:.4f}")
            else:
                rospy.logerr("[TF Adjuster] Not enough motion to compute heading offset (min 5m required).")

            self.offset_applied = True
            self.fix_session_active = False

    def publish_tf(self, yaw):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "gps"

        t.transform.translation.x = self.tf_data.get("base_gps_x_tf", 0.0)
        t.transform.translation.y = self.tf_data.get("base_gps_y_tf", 0.0)
        t.transform.translation.z = self.tf_data.get("base_gps_z_tf", 0.0)

        pitch = self.tf_data.get("base_gps_pitch_tf", 0.0)
        roll = self.tf_data.get("base_gps_roll_tf", 0.0)
        quat = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.br.sendTransform(t)

    def load_tf_yaml(self):
        try:
            with open(YAML_PATH, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            rospy.logerr(f"Failed to read TF YAML: {e}")
            return {}

    def write_tf_yaml(self, data):
        try:
            with open(YAML_PATH, 'w') as f:
                yaml.safe_dump(data, f)
        except Exception as e:
            rospy.logerr(f"Failed to write TF YAML: {e}")

    def normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

if __name__ == "__main__":
    try:
        ManualHeadingOffsetCorrector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
