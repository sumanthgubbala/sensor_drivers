#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf2_ros
import tf
import numpy as np
import time

def quat_to_matrix(q):
    return tf.transformations.quaternion_matrix([q[0], q[1], q[2], q[3]])

def pose_to_matrix(position, orientation):
    m = quat_to_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
    m[0:3, 3] = [position.x, position.y, position.z]
    return m

def transform_to_matrix(trans):
    q = trans.rotation
    t = trans.translation
    m = quat_to_matrix([q.x, q.y, q.z, q.w])
    m[0:3, 3] = [t.x, t.y, t.z]
    return m

class LocalizationValidator(object):
    def __init__(self):
        rospy.init_node('localization_validator', anonymous=False)

        self.rate_hz = float(rospy.get_param('~check_rate_hz', 5.0))
        self.freshness_s = float(rospy.get_param('~freshness_timeout_s', 1.0))
        self.pos_tol = float(rospy.get_param('~position_tolerance_m', 0.5))
        self.success_threshold = int(rospy.get_param('~success_count', 3))
        self.failure_threshold = int(rospy.get_param('~failure_count', 3))

        self.gps_fix = None
        self.gps_fix_time = 0.0
        self.vehicle_pose = None
        self.vehicle_pose_time = 0.0
        self.odom_filtered = None
        self.odom_filtered_time = 0.0

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.success_count = 0
        self.failure_count = 0
        self.current_valid = False

        self.param_name = '/localization_valid'
        rospy.set_param(self.param_name, False)

        self.bool_pub = rospy.Publisher('/localization_valid_bool', Bool, queue_size=1)

        rospy.Subscriber('/gps/fix', NavSatFix, self.cb_gps)
        rospy.Subscriber('/ML/VehiclePose', PoseStamped, self.cb_vehicle_pose)
        rospy.Subscriber('/odometry/filtered', Odometry, self.cb_odom_filtered)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.timer_cb)

    def cb_gps(self, msg):
        self.gps_fix = msg
        self.gps_fix_time = rospy.Time.now().to_sec()

    def cb_vehicle_pose(self, msg):
        self.vehicle_pose = msg
        self.vehicle_pose_time = rospy.Time.now().to_sec()

    def cb_odom_filtered(self, msg):
        self.odom_filtered = msg
        self.odom_filtered_time = rospy.Time.now().to_sec()

    def is_fresh(self, msg_time):
        if msg_time is None:
            return False
        return (rospy.Time.now().to_sec() - msg_time) <= self.freshness_s

    def compute_map_transformed_position(self, odom_msg):
        try:
            stamp = odom_msg.header.stamp
            if stamp == rospy.Time(0):
                lookup_time = rospy.Time.now()
            else:
                lookup_time = stamp
            t_stamped = self.tf_buffer.lookup_transform('map', 'odom', lookup_time, rospy.Duration(0.5))
            map_to_odom_mat = transform_to_matrix(t_stamped.transform)
            odom_pose = odom_msg.pose.pose
            odom_mat = pose_to_matrix(odom_pose.position, odom_pose.orientation)
            map_to_base = np.dot(map_to_odom_mat, odom_mat)
            tx = float(map_to_base[0, 3])
            ty = float(map_to_base[1, 3])
            rot_mat = map_to_base[0:3, 0:3]
            quat = tf.transformations.quaternion_from_matrix(map_to_base)
            return (tx, ty, quat)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return None

    def timer_cb(self, event):
        gps_ok = self.gps_fix is not None and self.is_fresh(self.gps_fix_time)
        veh_ok = self.vehicle_pose is not None and self.is_fresh(self.vehicle_pose_time)
        odom_ok = self.odom_filtered is not None and self.is_fresh(self.odom_filtered_time)

        if not (gps_ok and veh_ok and odom_ok):
            self.failure_count += 1
            self.success_count = 0
            if self.failure_count >= self.failure_threshold:
                if self.current_valid:
                    rospy.set_param(self.param_name, False)
                    self.bool_pub.publish(Bool(False))
                    self.current_valid = False
            return

        comp = self.compute_map_transformed_position(self.odom_filtered)
        if comp is None:
            self.failure_count += 1
            self.success_count = 0
            if self.failure_count >= self.failure_threshold:
                if self.current_valid:
                    rospy.set_param(self.param_name, False)
                    self.bool_pub.publish(Bool(False))
                    self.current_valid = False
            return

        map_x, map_y, map_quat = comp
        veh_x = float(self.vehicle_pose.pose.position.x)
        veh_y = float(self.vehicle_pose.pose.position.y)

        dx = abs(map_x - veh_x)
        dy = abs(map_y - veh_y)

        if dx <= self.pos_tol and dy <= self.pos_tol:
            self.success_count += 1
            self.failure_count = 0
            if self.success_count >= self.success_threshold:
                if not self.current_valid:
                    rospy.set_param(self.param_name, True)
                    self.bool_pub.publish(Bool(True))
                    self.current_valid = True
        else:
            self.failure_count += 1
            self.success_count = 0
            if self.failure_count >= self.failure_threshold:
                if self.current_valid:
                    rospy.set_param(self.param_name, False)
                    self.bool_pub.publish(Bool(False))
                    self.current_valid = False

        rospy.logdebug_throttle(5.0, "map_pos=(%.6f,%.6f) veh_pos=(%.6f,%.6f) dx=%.4f dy=%.4f tol=%.3f",
                                map_x, map_y, veh_x, veh_y, dx, dy, self.pos_tol)

if __name__ == "__main__":
    node = LocalizationValidator()
    rospy.spin()
