#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import tf
import os
import csv
import math
from nav_msgs.msg import OccupancyGrid

class OutdoorMapping:
    def __init__(self):
        rospy.init_node("ab_line_ml_node", anonymous=False)

        self.map_margin = rospy.get_param("/map_margin", 5)
        self.map_resolution = rospy.get_param("/map_resolution", 0.05)
        self.map2odom = rospy.get_param("/map2odom", 0.0)
        self.line_thickness = rospy.get_param("/line_thickness", 3)
        self.map_save_directory = rospy.get_param("/map_save_directory", "/home/kiosk/config/")
        # self.map_save_directory = rospy.get_param("/map_save_directory", "/home/fluxauto/ua/ceres_dep_dev_ws/src/ceres_pp_stack/ceres_pp/planner_core/config/")
        self.save_map = rospy.get_param("/save_map", False)
        self.map_name = rospy.get_param("/map_name", "ceres_farm_map")

        rospy.loginfo(f"Parameters :\n"
                      f"map_margin: {self.map_margin}\n"
                      f"map_resolution: {self.map_resolution}\n"
                      f"map2odom: {self.map2odom}\n"
                      f"line_thickness: {self.line_thickness}\n"
                      f"map_save_directory: {self.map_save_directory}\n"
                      f"map_name: {self.map_name}\n"
                      f"save_map: {self.save_map}")

        self.position_list = []
        self.rotated_position_list = []
        self.obstacle_list = []  # each obstacle is a list of 4 corner tuples
        self.rotated_obstacle_list = []

        original_boundary_location = os.path.join(self.map_save_directory, self.map_name)
        os.makedirs(original_boundary_location, exist_ok=True)

        self.load_boundary_from_csv(os.path.join(original_boundary_location, "original_boundary.csv"))
        self.load_obstacles_from_csv(os.path.join(original_boundary_location, "obstacle_position.csv"))
        self.update_rotated_positions()

        self.map_pub = rospy.Publisher("/map_original", OccupancyGrid, queue_size=1, latch=True)
        self.rotated_map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)

        self.map_timer = rospy.Timer(rospy.Duration(1.0), self.publish_maps)
        self.param_timer = rospy.Timer(rospy.Duration(0.1), self.check_params_update)

    def load_boundary_from_csv(self, file_path):
        if not os.path.exists(file_path):
            rospy.logerr(f"CSV file not found: {file_path}")
            return
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            self.position_list = [(float(row[0]), float(row[1])) for row in reader]

    def load_obstacles_from_csv(self, file_path):
        if not os.path.exists(file_path):
            rospy.logwarn(f"Obstacle CSV file not found: {file_path}")
            return

        all_points = []
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) != 2:
                    rospy.logwarn(f"Invalid obstacle row: {row}")
                    continue
                try:
                    x, y = float(row[0]), float(row[1])
                    all_points.append((x, y))
                except ValueError:
                    rospy.logwarn(f"Non-numeric obstacle point: {row}")
                    continue

        # Group every 4 points as one obstacle
        self.obstacle_list = []
        for i in range(0, len(all_points), 4):
            if i + 4 <= len(all_points):
                self.obstacle_list.append(all_points[i:i + 4])
            else:
                rospy.logwarn(f"Incomplete obstacle definition at end of file: {all_points[i:]}")


    def update_rotated_positions(self):
        angle = np.radians(self.map2odom)
        cos_a, sin_a = np.cos(angle), np.sin(angle)

        def rotate_point(x, y):
            return cos_a * x - sin_a * y, sin_a * x + cos_a * y

        self.rotated_position_list = [rotate_point(x, y) for x, y in self.position_list]
        self.rotated_obstacle_list = [
            [rotate_point(x, y) for x, y in obstacle] for obstacle in self.obstacle_list
        ]

    def check_params_update(self, event):
        new_map2odom = rospy.get_param("/map2odom", self.map2odom)
        new_line_thickness = rospy.get_param("/line_thickness", self.line_thickness)
        new_map_margin = rospy.get_param("/map_margin", self.map_margin)
        save_map = rospy.get_param("/save_map", False)

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

        if save_map:
            rospy.loginfo(f"Saving map")
            self.save_map_files()
            self.save_geofence()
            rospy.set_param("/save_map", False)

    def create_map(self, position_list, obstacle_list=[], flip_y=False):
        if not position_list:
            return None, None, None

        all_x = [x for x, _ in position_list] + [x for obs in obstacle_list for x, _ in obs]
        all_y = [y for _, y in position_list] + [y for obs in obstacle_list for _, y in obs]

        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

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

        for obstacle in obstacle_list:
            obstacle_pixels = [
                (int((x - min_x + self.map_margin) / self.map_resolution),
                 int((y - min_y + self.map_margin) / self.map_resolution))
                for x, y in obstacle
            ]
            cv2.fillPoly(grid_map, [np.array(obstacle_pixels, dtype=np.int32)], 0)

        if flip_y:
            grid_map = cv2.flip(grid_map, 0)

        return grid_map, min_x - self.map_margin, min_y - self.map_margin

    def publish_maps(self, event):
        original_map, origin_x, origin_y = self.create_map(self.position_list, self.obstacle_list)
        rotated_map, rotated_origin_x, rotated_origin_y = self.create_map(self.rotated_position_list, self.rotated_obstacle_list)

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

    def save_map_files(self):
        save_map, save_origin_x, save_origin_y = self.create_map(self.rotated_position_list, self.rotated_obstacle_list, flip_y=True)
        map_dir = os.path.join(self.map_save_directory, self.map_name)
        os.makedirs(map_dir, exist_ok=True)

        if save_map is not None:
            pgm_filename = os.path.join(self.map_save_directory, "ceres_farm_map.pgm")
            pgm_filename_dir = os.path.join(map_dir, "ceres_farm_map.pgm")
            yaml_filename = os.path.join(self.map_save_directory, "ceres_farm_map.yaml")
            yaml_filename_dir = os.path.join(map_dir, "ceres_farm_map.yaml")
            cv2.imwrite(pgm_filename, save_map)
            cv2.imwrite(pgm_filename_dir, save_map)

            with open(yaml_filename, 'w') as yaml_file:
                yaml_file.write(f"image: ceres_farm_map.pgm\n")
                yaml_file.write(f"resolution: {self.map_resolution}\n")
                yaml_file.write(f"origin: [{save_origin_x}, {save_origin_y}, 0.0]\n")
                yaml_file.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
            with open(yaml_filename_dir, 'w') as yaml_file_dir:
                yaml_file_dir.write(f"image: ceres_farm_map.pgm\n")
                yaml_file_dir.write(f"resolution: {self.map_resolution}\n")
                yaml_file_dir.write(f"origin: [{save_origin_x}, {save_origin_y}, 0.0]\n")
                yaml_file_dir.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

    def save_geofence(self):
        map_dir = os.path.join(self.map_save_directory, self.map_name)
        os.makedirs(map_dir, exist_ok=True)
        rotated_csv_filename = os.path.join(map_dir, "safe_boundary.csv")

        if not self.rotated_position_list:
            rospy.logwarn("No geofence data available to save.")
            return

        sparse_geofence_data = [self.rotated_position_list[0]]

        for x, y in self.rotated_position_list[1:]:
            last_x, last_y = sparse_geofence_data[-1]
            distance = math.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)
            if distance >= 2.0:
                sparse_geofence_data.append((x, y))

        with open(rotated_csv_filename, 'w', newline='') as csvfile:
            writer2 = csv.writer(csvfile)
            writer2.writerows(sparse_geofence_data)

        rospy.loginfo(f"Geofence data saved to {rotated_csv_filename}")

if __name__ == "__main__":
    OutdoorMapping()
    rospy.spin()

