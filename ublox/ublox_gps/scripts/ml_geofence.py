#!/usr/bin/env python

import rospy
import csv
from shapely.geometry import Point, Polygon, MultiPolygon
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point as GeoPoint

CSV_FILE_PATH = "/home/ubuntu/ceres_dep_dev_ws/src/ceres_pp_stack/ceres_pp/planner_core/config/ceres_farm_map/safe_boundary.csv"

def load_geofence_from_csv(file_path):
    polygon_coords = []
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            try:
                x, y = float(row[0]), float(row[1])
                polygon_coords.append((x, y))
            except ValueError:
                rospy.logwarn(f"Invalid row in CSV: {row}")
    return Polygon(polygon_coords), polygon_coords

safe_zone_polygon, polygon_coords = load_geofence_from_csv(CSV_FILE_PATH)

lift_zone_polygon = None
drop_zone_polygon = None
geofence_polygon = None

marker_pub = None

def generate_fence(fence1, fence2):
    poly1 = Polygon(fence1)
    poly2 = Polygon(fence2)
    overlap = poly1.intersection(poly2)
    
    if overlap.is_empty:
        return []
    
    if isinstance(overlap, MultiPolygon):
        overlap = max(overlap.geoms, key=lambda p: p.area)
    
    return list(overlap.exterior.coords) if overlap.exterior else []

def update_markers():
    global lift_zone_polygon, drop_zone_polygon, geofence_polygon
    lift_offset = rospy.get_param('/lift_offset', 9)
    drop_offset = rospy.get_param('/drop_offset', 7)
    geofence_offset = rospy.get_param('/geofence_offset', 3)
    
    lift_fence1 = [(x, y + lift_offset) for x, y in polygon_coords] if lift_offset is not None else []
    lift_fence2 = [(x, y - lift_offset) for x, y in polygon_coords] if lift_offset is not None else []
    drop_fence1 = [(x, y + drop_offset) for x, y in polygon_coords] if drop_offset is not None else []
    drop_fence2 = [(x, y - drop_offset) for x, y in polygon_coords] if drop_offset is not None else []
    
    lift_zone_polygon = Polygon(generate_fence(lift_fence1, lift_fence2)) if lift_offset is not None else None
    drop_zone_polygon = Polygon(generate_fence(drop_fence1, drop_fence2)) if drop_offset is not None else None
    geofence_polygon = safe_zone_polygon.buffer(geofence_offset) if geofence_offset is not None else None
    
    publish_all_markers()

def publish_all_markers():
    global marker_pub
    
    safe_zone_marker = create_marker(polygon_coords, (1.0, 0.0, 0.0), "safe_zone_marker")
    marker_pub.publish(safe_zone_marker)
    
    if lift_zone_polygon:
        lift_marker = create_marker(list(lift_zone_polygon.exterior.coords), (0.0, 1.0, 0.0), "lift_zone_marker")
        marker_pub.publish(lift_marker)
    
    if geofence_polygon:
        geofence_marker = create_marker(list(geofence_polygon.exterior.coords), (1.0, 1.0, 1.0), "geofence_marker")
        marker_pub.publish(geofence_marker)
    
    if drop_zone_polygon:
        drop_marker = create_marker(list(drop_zone_polygon.exterior.coords), (0.0, 0.0, 1.0), "drop_zone_marker")
        marker_pub.publish(drop_marker)

def create_marker(coords, color, marker_ns):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.3
    marker.color.a = 1.0
    marker.color.r, marker.color.g, marker.color.b = color
    marker.ns = marker_ns
    marker.pose.orientation.w = 1.0
    
    for x, y in coords:
        p = GeoPoint()
        p.x, p.y, p.z = x, y, 0
        marker.points.append(p)
    marker.points.append(marker.points[0])
    return marker

def vehicle_callback(msg):
    global lift_zone_polygon, drop_zone_polygon
    current_position = Point(msg.pose.position.x, msg.pose.position.y)
    
    if geofence_polygon and geofence_polygon.contains(current_position):
        rospy.set_param('/ML/geofence', True)
    else:
        rospy.set_param('/ML/geofence', False)
    
    if safe_zone_polygon and safe_zone_polygon.contains(current_position):
        rospy.set_param('/ML/safe_zone', True)
    else:
        rospy.set_param('/ML/safe_zone', False)
    
    if lift_zone_polygon and lift_zone_polygon.contains(current_position):
        rospy.set_param('/ML/implement_lift', True)
    else:
        rospy.set_param('/ML/implement_lift', False)
    
    if drop_zone_polygon and drop_zone_polygon.contains(current_position):
        rospy.set_param('/ML/implement_drop', True)
    else:
        rospy.set_param('/ML/implement_drop', False)

def geofence_checker():
    global marker_pub
    rospy.init_node('geofence_checker', anonymous=True)
    marker_pub = rospy.Publisher('geofence_marker', Marker, queue_size=10)
    rospy.Subscriber('/ML/VehiclePose', PoseStamped, vehicle_callback)
    update_markers()
    rospy.spin()

if __name__ == '__main__':
    try:
        geofence_checker()
    except rospy.ROSInterruptException:
        pass

