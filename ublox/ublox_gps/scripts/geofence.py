#!/usr/bin/env python3

import rospy
from shapely.geometry import Point, Polygon
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as GeoPoint

polygon_coords = [
    (13.1202014, 77.5274803),
    (13.1200992, 77.5278266),
    (13.1199727, 77.5281791),
    (13.1199122, 77.5282090),
    (13.1197604, 77.5281624),
    (13.1197182, 77.5280831),
    (13.1197676, 77.5278457),
    (13.1199176, 77.5274156),
    (13.1199530, 77.5273946)
    #(13.0601795, 77.5506232),
    #(13.0600715, 77.5505613),
    #(13.0600905, 77.5510694),
    #(13.0602428, 77.5511223)
    # (13.060212, 77.550937),
    # (13.060572, 77.550950),
    # (13.060562, 77.551102),
    # (13.060171, 77.551295),
    # (13.059984, 77.551053),
    # (13.060038, 77.550546),
    # (13.060219, 77.550577)
]

polygon = Polygon(polygon_coords)

polygon_marker = Marker()
gps_marker = Marker()

def setup_markers():
    global polygon_marker, gps_marker
    polygon_marker.header.frame_id = "map"
    polygon_marker.type = Marker.LINE_STRIP
    polygon_marker.action = Marker.ADD
    polygon_marker.scale.x = 0.1
    polygon_marker.color.a = 1.0
    polygon_marker.color.r = 1.0
    polygon_marker.pose.orientation.w = 1.0

    for lat, lon in polygon_coords:
        p = GeoPoint()
        p.x = lon
        p.y = lat
        p.z = 0
        polygon_marker.points.append(p)
    polygon_marker.points.append(polygon_marker.points[0])

    gps_marker.header.frame_id = "map"
    gps_marker.type = Marker.SPHERE
    gps_marker.action = Marker.ADD
    gps_marker.scale.x = 0.5
    gps_marker.scale.y = 0.5
    gps_marker.scale.z = 0.5
    gps_marker.color.a = 1.0
    gps_marker.color.g = 1.0
    gps_marker.pose.orientation.w = 1.0


def gps_callback(msg):
    current_position = Point(msg.latitude, msg.longitude)

    gps_marker.pose.position.x = msg.longitude
    gps_marker.pose.position.y = msg.latitude

    marker_pub.publish(polygon_marker)
    marker_pub.publish(gps_marker)

    if polygon.contains(current_position):
        rospy.set_param('/ML/geofence', False)
        rospy.loginfo("Vehicle is inside the geofence.")
    else:
        rospy.set_param('/ML/geofence', True)
        rospy.loginfo("Vehicle is outside the geofence.")


def geofence_checker():
    global marker_pub
    rospy.init_node('geofence_checker', anonymous=True)

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    setup_markers()

    rospy.Subscriber('/gps/fix', NavSatFix, gps_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        geofence_checker()
    except rospy.ROSInterruptException:
        pass

