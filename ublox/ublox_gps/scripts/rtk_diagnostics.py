#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time

COVARIANCE_THRESHOLD = 0.0002
DATA_TIMEOUT_SECONDS = 2.0  # Time to wait before assuming no data

class GpsCovarianceMonitor:
    def __init__(self):
        rospy.init_node('gps_covariance_monitor')
        rospy.Subscriber('/gps/fix', NavSatFix, self.fix_callback)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.last_fix_time = None
        rospy.loginfo("GPS Covariance Monitor started.")
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_timeout)

    def fix_callback(self, msg):
        self.last_fix_time = rospy.Time.now()

        diag_msg = DiagnosticArray()
        diag_status = DiagnosticStatus()
        diag_status.name = "GPS Covariance Monitor"
        diag_status.hardware_id = "simpleRTK2B"
        
        # Extract diagonal of position_covariance matrix
        cov_x = msg.position_covariance[0]
        cov_y = msg.position_covariance[4]
        cov_z = msg.position_covariance[8]

        exceeds_threshold = any(cov > COVARIANCE_THRESHOLD for cov in (cov_x, cov_y, cov_z))

        if exceeds_threshold:
            diag_status.level = DiagnosticStatus.ERROR
            diag_status.message = "Position covariance exceeds threshold"
        else:
            diag_status.level = DiagnosticStatus.OK
            diag_status.message = "Position covariance within acceptable limits"

        diag_status.values = [
            KeyValue(key="Covariance X (m^2)", value=str(cov_x)),
            KeyValue(key="Covariance Y (m^2)", value=str(cov_y)),
            KeyValue(key="Covariance Z (m^2)", value=str(cov_z)),
            KeyValue(key="Threshold (m^2)", value=str(COVARIANCE_THRESHOLD))
        ]

        diag_msg.status.append(diag_status)
        diag_msg.header.stamp = rospy.Time.now()
        self.diag_pub.publish(diag_msg)

    def check_timeout(self, event):
        diag_msg = DiagnosticArray()
        diag_status = DiagnosticStatus()
        diag_status.name = "GPS Covariance Monitor"
        diag_status.hardware_id = "simpleRTK2B"

        now = rospy.Time.now()

        # Case 1: Topic exists but no messages received recently
        if self.last_fix_time is not None:
            elapsed = (now - self.last_fix_time).to_sec()
            if elapsed > DATA_TIMEOUT_SECONDS:
                diag_status.level = DiagnosticStatus.ERROR
                diag_status.message = "No /gps/fix data received in last {}s".format(DATA_TIMEOUT_SECONDS)
            else:
                return  # Data is arriving, no issue to report

        # Case 2: /gps/fix topic doesn't exist at all
        elif '/gps/fix' not in [t for t, _ in rospy.get_published_topics()]:
            diag_status.level = DiagnosticStatus.ERROR
            diag_status.message = "/gps/fix topic not available"
        else:
            diag_status.level = DiagnosticStatus.WARN
            diag_status.message = "/gps/fix topic available but no data received yet"

        diag_status.values = [
            KeyValue(key="Last Fix Time", value=str(self.last_fix_time) if self.last_fix_time else "None"),
            KeyValue(key="Timeout (s)", value=str(DATA_TIMEOUT_SECONDS))
        ]

        diag_msg.status.append(diag_status)
        diag_msg.header.stamp = now
        self.diag_pub.publish(diag_msg)

if __name__ == '__main__':
    try:
        monitor = GpsCovarianceMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

