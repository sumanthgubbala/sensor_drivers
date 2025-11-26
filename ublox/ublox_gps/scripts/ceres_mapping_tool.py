#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import tf
import os
import csv
import math
from nav_msgs.msg import Odometry, OccupancyGrid

class OutdoorMapping:
    def __init__(self):
        rospy.init_node("outdoor_mapping_node", anonymous=False)

        self.map_margin = rospy.get_param("/map_margin", 5)
        self.map_resolution = rospy.get_param("/map_resolution", 0.05)
        self.map2odom = rospy.get_param("/map2odom", 0.0)
        self.line_thickness = rospy.get_param("/line_thickness", 3)
        self.map_save_directory = rospy.get_param("/map_save_directory", "/home/kiosk/config/")
        # self.map_save_directory = rospy.get_param("/map_save_directory", "/home/fluxauto/ua/ceres_dep_dev_ws/src/ceres_pp_stack/ceres_pp/planner_core/config/")
        self.default_map_directory = rospy.get_param("/default_map_directory", "/home/kiosk/config/")
        # self.default_map_directory = rospy.get_param("/default_map_directory", "/home/fluxauto/ua/ceres_dep_dev_ws/src/ceres_pp_stack/ceres_pp/planner_core/config/")
        self.save_map_name = rospy.get_param("/save_map_with_name", "")
        self.record_map = rospy.get_param("/record_map", True)

        # Read offset parameters once at startup
        self.implement_width = rospy.get_param("/implement_width", 3.0)
        self.record_point_direction = rospy.get_param("/record_point_direction", "left")

        rospy.loginfo(f"Parameters :\n"
                      f"map_margin: {self.map_margin}\n"
                      f"map_resolution: {self.map_resolution}\n"
                      f"map2odom: {self.map2odom}\n"
                      f"line_thickness: {self.line_thickness}\n"
                      f"map_save_directory: {self.map_save_directory}\n"
                      f"save_map_name: {self.save_map_name}\n"
                      f"implement_width: {self.implement_width}\n"
                      f"record_point_direction: {self.record_point_direction}")

        self.position_list = []
        self.rotated_position_list = []

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self.map_pub = rospy.Publisher("/map_original", OccupancyGrid, queue_size=1, latch=True)
        self.rotated_map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)

        
        self.map_timer = rospy.Timer(rospy.Duration(1.0), self.publish_maps)
        self.param_timer = rospy.Timer(rospy.Duration(0.1), self.check_params_update)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.record_map = rospy.get_param("/record_map", True)
        if not self.record_map:
            return

        orientation = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        offset_sign = 1.0 if self.record_point_direction.lower() == "left" else -1.0
        offset_magnitude = self.implement_width / 2.0

        offset_x = -offset_sign * offset_magnitude * math.sin(yaw)
        offset_y =  offset_sign * offset_magnitude * math.cos(yaw)

        offset_point_x = x + offset_x
        offset_point_y = y + offset_y

        self.position_list.append((offset_point_x, offset_point_y))
        self.update_rotated_positions()

    def check_params_update(self, event):
        new_map2odom = rospy.get_param("/map2odom", self.map2odom)
        new_line_thickness = rospy.get_param("/line_thickness", self.line_thickness)
        new_map_margin = rospy.get_param("/map_margin", self.map_margin)
        new_save_map = rospy.get_param("/save_map_with_name", "")

        if new_map2odom != self.map2odom:
            rospy.loginfo(f"map2odom changed: {self.map2odom} → {new_map2odom}")
            self.map2odom = new_map2odom
            self.update_rotated_positions()

        if new_line_thickness != self.line_thickness:
            rospy.loginfo(f"line_thickness changed: {self.line_thickness} → {new_line_thickness}")
            self.line_thickness = new_line_thickness

        if new_map_margin != self.map_margin:
            rospy.loginfo(f"map_margin changed: {self.map_margin} → {new_map_margin}")
            self.map_margin = new_map_margin

        if new_save_map:
            rospy.loginfo("Saving map and safe_boundary data...")
            self.save_map_files(new_save_map)
            self.save_geofence(new_save_map)
            self.save_map2odom(new_save_map)
            rospy.set_param("/save_map_with_name", "")

    def update_rotated_positions(self):
        angle = np.radians(self.map2odom)
        cos_a, sin_a = np.cos(angle), np.sin(angle)

        self.rotated_position_list = [
            (cos_a * x - sin_a * y, sin_a * x + cos_a * y) for x, y in self.position_list
        ]

    def create_map(self, position_list, flip_y=False):
        if not position_list:
            return None, None, None

        x_values, y_values = zip(*position_list)
        min_x, max_x = min(x_values), max(x_values)
        min_y, max_y = min(y_values), max(y_values)

        width = int((max_x - min_x + 2 * self.map_margin) / self.map_resolution)
        height = int((max_y - min_y + 2 * self.map_margin) / self.map_resolution)

        grid_map = np.full((height, width), 192, dtype=np.uint8)

        points = [
            (int((x - min_x + self.map_margin) / self.map_resolution),
             int((y - min_y + self.map_margin) / self.map_resolution))
            for x, y in position_list
        ]

        for i in range(len(points) - 1):
            cv2.line(grid_map, points[i], points[i + 1], 0, self.line_thickness)

        if len(points) > 2:
            cv2.line(grid_map, points[0], points[-1], 0, self.line_thickness)

        mask = np.zeros_like(grid_map, dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(points, dtype=np.int32)], 255)
        grid_map[mask == 255] = 255

        if flip_y:
            grid_map = cv2.flip(grid_map, 0)

        return grid_map, min_x - self.map_margin, min_y - self.map_margin

    def compute_and_publish_map_stats(self):
        grid, _, _ = self.create_map(self.position_list)
        if grid is None:
            rospy.set_param("/map_x_dimension", 0.0)
            rospy.set_param("/map_y_dimension", 0.0)
            rospy.set_param("/map_enclosed_area", 0.0)
            return

        h, w = grid.shape
        map_w_m = w * self.map_resolution
        map_h_m = h * self.map_resolution

        x_dim = max(map_w_m - 2.0 * self.map_margin, 0.0)
        y_dim = max(map_h_m - 2.0 * self.map_margin, 0.0)

        area = 0.0
        if len(self.position_list) >= 3:
            cnt = np.array(self.position_list, dtype=np.float32).reshape((-1, 1, 2))
            area = float(abs(cv2.contourArea(cnt)))  # m^2

        rospy.set_param("/map_x_dimension", float(x_dim))
        rospy.set_param("/map_y_dimension", float(y_dim))
        rospy.set_param("/map_enclosed_area", float(area))


    def publish_maps(self, event):
        original_map, origin_x, origin_y = self.create_map(self.position_list)
        rotated_map, rotated_origin_x, rotated_origin_y = self.create_map(self.rotated_position_list)
        self.compute_and_publish_map_stats()

        if original_map is not None:
            self.publish_occupancy_grid(original_map, self.map_pub, origin_x, origin_y)
        if rotated_map is not None:
            self.publish_occupancy_grid(rotated_map, self.rotated_map_pub, rotated_origin_x, rotated_origin_y)

    def publish_occupancy_grid(self, grid, publisher, origin_x, origin_y):
        height, width = grid.shape
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = width
        grid_msg.info.height = height
        grid_msg.info.origin.position.x = origin_x
        grid_msg.info.origin.position.y = origin_y

        ros_grid = np.full(grid.shape, -1, dtype=np.int8)
        ros_grid[grid == 192] = -1
        ros_grid[grid == 0] = 100
        ros_grid[grid == 255] = 0

        grid_msg.data = ros_grid.flatten().tolist()
        publisher.publish(grid_msg)

    def save_map2odom(self, map_name):
        # data = {
        #     'map2odom': 0.0,
        #     'map2odomRad': 0.0
        # }
        map_dir = os.path.join(self.map_save_directory, map_name)
        map2odom_yaml = os.path.join(map_dir, "map2odom.yaml")
        with open(map2odom_yaml, 'w') as m2ofile:
            m2ofile.write(f"map2odom: 0.0\n")
            m2ofile.write(f"map2odomRad: 0.0\n")

    def save_map_files(self, map_name):
        map_image, save_origin_x, save_origin_y = self.create_map(self.rotated_position_list, flip_y=True)
        save_map_orig, save_origin_x_orig, save_origin_y_orig = self.create_map(self.position_list, flip_y=True)
        map_dir = os.path.join(self.map_save_directory, map_name)
        os.makedirs(map_dir, exist_ok=True)
        
        if map_image is not None:
            # pgm_filename = os.path.join(self.map_save_directory, "ceres_farm_map.pgm")
            pgm_filename = os.path.join(map_dir, "ceres_farm_map.pgm")
            pgm_filename_default = os.path.join(self.default_map_directory, "ceres_farm_map.pgm")
            pgm_filename_orig = os.path.join(map_dir, "ceres_farm_map_orig.pgm")
            # yaml_filename = os.path.join(self.map_save_directory, "ceres_farm_map.yaml")
            yaml_filename = os.path.join(map_dir, "ceres_farm_map.yaml")
            yaml_filename_default = os.path.join(self.default_map_directory, "ceres_farm_map.yaml")
            yaml_filename_orig = os.path.join(map_dir, "ceres_farm_map_orig.yaml")
            cv2.imwrite(pgm_filename, map_image)
            cv2.imwrite(pgm_filename_default, map_image)
            cv2.imwrite(pgm_filename_orig, save_map_orig)

            with open(yaml_filename, 'w') as yaml_file:
                yaml_file.write(f"image: ceres_farm_map.pgm\n")
                yaml_file.write(f"resolution: {self.map_resolution}\n")
                yaml_file.write(f"origin: [{save_origin_x}, {save_origin_y}, 0.0]\n")
                yaml_file.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
            
            with open(yaml_filename_default, 'w') as yaml_file_default:
                yaml_file_default.write(f"image: ceres_farm_map.pgm\n")
                yaml_file_default.write(f"resolution: {self.map_resolution}\n")
                yaml_file_default.write(f"origin: [{save_origin_x}, {save_origin_y}, 0.0]\n")
                yaml_file_default.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

            with open(yaml_filename_orig, 'w') as yaml_file_orig:
                yaml_file_orig.write(f"image: ceres_farm_map_orig.pgm\n")
                yaml_file_orig.write(f"resolution: {self.map_resolution}\n")
                yaml_file_orig.write(f"origin: [{save_origin_x_orig}, {save_origin_y_orig}, 0.0]\n")
                yaml_file_orig.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

    def save_geofence(self, map_name):
        map_dir = os.path.join(self.map_save_directory, map_name)
        os.makedirs(map_dir, exist_ok=True)
        rotated_csv_filename = os.path.join(map_dir, "safe_boundary.csv")
        original_csv_filename = os.path.join(map_dir, "original_boundary.csv")

        if not self.rotated_position_list:
            rospy.logwarn("No geofence data available to save.")
            return

        sparse_geofence_data = []
        sparse_geofence_data.append(self.rotated_position_list[0])

        for x, y in self.rotated_position_list[1:]:
            last_x, last_y = sparse_geofence_data[-1]
            distance = math.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)

            if distance >= 2.0:
                sparse_geofence_data.append((x, y))

        with open(original_csv_filename, 'w', newline='') as origcsvfile:
            writer1 = csv.writer(origcsvfile)
            writer1.writerows(self.position_list)

        with open(rotated_csv_filename, 'w', newline='') as csvfile:
            writer2 = csv.writer(csvfile)
            writer2.writerows(sparse_geofence_data)

        rospy.loginfo(f"Geofence data saved to {rotated_csv_filename}")

if __name__ == "__main__":
    OutdoorMapping()
    rospy.spin()


