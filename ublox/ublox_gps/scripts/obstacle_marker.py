#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import tf
import os
from PIL import Image
import yaml
import std_msgs.msg

class ObstacleMarker:
    def __init__(self):
        rospy.init_node("obstacle_marker")

        self.map = None
        self.map_info = None
        self.pose = None
        self.map_data = None
        self.obstacle_corners = []

        # Subscribers
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/ML/VehiclePose", PoseStamped, self.pose_callback)

        # Publisher
        self.map_pub = rospy.Publisher("/obstacle_map", OccupancyGrid, queue_size=1, latch=True)

        rospy.Timer(rospy.Duration(1.0), self.timer_callback)
        rospy.spin()

    def map_callback(self, msg):
        self.map = msg
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def pose_callback(self, msg):
        self.pose = msg

    def timer_callback(self, event):
        if not self.map or not self.pose:
            return

        if rospy.get_param("/mark_obstacles", False):
            self.mark_obstacle()
            rospy.set_param("/mark_obstacles", False)
            self.publish_obstacle_map()

        if rospy.get_param("/obstacle_marking_done", False):
            self.save_map()
            rospy.set_param("/obstacle_marking_done", False)

    def mark_obstacle(self):
        a = rospy.get_param("/a", 2.0)
        l = rospy.get_param("/l", 1.3)
        b = rospy.get_param("/b", 1.9)

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        orientation_q = self.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

        front_x = x + a * np.cos(yaw)
        front_y = y + a * np.sin(yaw)

        length_vec = np.array([np.cos(yaw), np.sin(yaw)])
        breadth_vec = np.array([-np.sin(yaw), np.cos(yaw)])

        half_l = l / 2.0
        half_b = b / 2.0

        corners = []
        for dx, dy in [(-half_l, -half_b), (-half_l, half_b), (half_l, half_b), (half_l, -half_b)]:
            offset = dx * length_vec + dy * breadth_vec
            corner_x = front_x + offset[0]
            corner_y = front_y + offset[1]
            corners.append((corner_x, corner_y))

        self.obstacle_corners.append(corners)

        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        for dx in np.arange(-half_l, half_l, resolution):
            for dy in np.arange(-half_b, half_b, resolution):
                offset = dx * length_vec + dy * breadth_vec
                obs_x = front_x + offset[0]
                obs_y = front_y + offset[1]

                map_x = int((obs_x - origin_x) / resolution)
                map_y = int((obs_y - origin_y) / resolution)

                if 0 <= map_x < self.map_info.width and 0 <= map_y < self.map_info.height:
                    self.map_data[map_y, map_x] = 100

        rospy.loginfo("Marked obstacle and recorded corner points.")

    def publish_obstacle_map(self):
        if self.map_data is None or self.map_info is None:
            return

        updated_map = OccupancyGrid()
        updated_map.header = std_msgs.msg.Header()
        updated_map.header.stamp = rospy.Time.now()
        updated_map.header.frame_id = "map"
        updated_map.info = self.map_info
        updated_map.data = self.map_data.flatten().tolist()
        self.map_pub.publish(updated_map)

    def save_map(self):
        save_path = rospy.get_param("/save_path", "/home/fluxauto/ua/ceres_dep_dev_ws/src/ceres_pp_stack/ceres_pp/planner_core/config/")
        os.makedirs(save_path, exist_ok=True)

        # Save image
        img = Image.new("L", (self.map_info.width, self.map_info.height))
        img_data = np.zeros_like(self.map_data, dtype=np.uint8)

        img_data[self.map_data == 0] = 254     # Free → white
        img_data[self.map_data == 100] = 0     # Obstacle → black
        img_data[self.map_data == -1] = 205    # Unknown → gray

        img.putdata(img_data.flatten())
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
        img.save(os.path.join(save_path, "ceres_farm_map.pgm"))

        # Save YAML
        map_yaml = {
            "image": "ceres_farm_map.pgm",
            "resolution": self.map_info.resolution,
            "origin": [self.map_info.origin.position.x,
                       self.map_info.origin.position.y,
                       self.get_yaw(self.map_info.origin.orientation)],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196
        }

        with open(os.path.join(save_path, "ceres_farm_map.yaml"), 'w') as f:
            yaml.dump(map_yaml, f)

        # Save corner points to CSV
        csv_path = rospy.get_param("/csv_save_path", "/home/fluxauto/ua/ceres_dep_dev_ws/src/ceres_pp_stack/ceres_pp/planner_core/config/ceres_farm_map/obstacle_position.csv")
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        with open(csv_path, 'w') as f:
            for corners in self.obstacle_corners:
                for x, y in corners:
                    f.write(f"{x},{y}\n")

        rospy.loginfo(f"Saved map to {save_path}")
        rospy.loginfo(f"Saved obstacle corners to {csv_path}")

    def get_yaw(self, q):
        _, _, yaw = tf.transformations.euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])
        return float(yaw)

if __name__ == '__main__':
    try:
        ObstacleMarker()
    except rospy.ROSInterruptException:
        pass

