#!/usr/bin/env python3
import rospy
import csv
import os
from ublox_msgs.msg import NavPVT

class GNSSLogger:
    def __init__(self):
        rospy.init_node("gnss_logger", anonymous=True)

        # File setup
        self.filename = os.path.expanduser("~/gnss_log.csv")
        self.csvfile = open(self.filename, "w", newline="")
        self.writer = csv.writer(self.csvfile)

        # Write header
        self.writer.writerow(["time", "lat", "lon", "hAcc_m", "fixType", "flags", "carrSoln"])

        # Subscriber
        rospy.Subscriber("/gps/navpvt", NavPVT, self.callback)

        rospy.loginfo(f"Logging GNSS data to {self.filename}")

    def callback(self, msg: NavPVT):
        # Extract values
        lat = msg.lat * 1e-7   # convert to degrees
        lon = msg.lon * 1e-7
        hAcc_m = msg.hAcc / 1000.0   # mm → m

        fixType = msg.fixType       # 0 = no fix, 2 = 2D, 3 = 3D, 4 = GNSS+Dead Reckoning, 5 = Time
        flags = msg.flags           # raw uint8 bitfield
        carrSoln = msg.flags2 & 0b11  # 0 = no carrier, 1 = float, 2 = fixed (from UBX spec)

        self.writer.writerow([rospy.get_time(), lat, lon, hAcc_m, fixType, flags, carrSoln])
        self.csvfile.flush()

    def spin(self):
        rospy.spin()
        self.csvfile.close()

if __name__ == "__main__":
    logger = GNSSLogger()
    logger.spin()

