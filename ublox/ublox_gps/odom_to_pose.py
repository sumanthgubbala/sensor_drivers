#! /usr/bin/python3

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
import numpy as np
from std_msgs.msg import Float32

pub_pose =  rospy.Publisher('/gps/pose',PoseStamped, queue_size=10)
pub_speed_fix =  rospy.Publisher('/gps/fix_speed',Float32, queue_size=10)
# pub_pose_utm =  rospy.Publisher('/gps/utm/pose',PoseStamped, queue_size=10)

pose = PoseStamped()
speed_ = Float32()

# imu_offset = np.radians(-93)

def velocity_callback(msg):
    global speed_
    speed_ = math.sqrt((msg.twist.twist.linear.x)**2+(msg.twist.twist.linear.y)**2)
    pub_speed_fix.publish(speed_*3.6)    

def ekf_odom_callback(odom):
    global pose 
    quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    # yaw += imu_offset
    yaw_print = yaw*180/np.pi
    if(yaw_print < 0):
    	yaw_print += 360
    # convert to PoseStamped
    pose.header = odom.header
    pose.header.frame_id = "map"
    pose.pose.position = odom.pose.pose.position
    quaternion2 = tf.transformations.quaternion_from_euler(euler[0], euler[1], yaw)
    # quaternion2 = tf.transformations.quaternion_from_euler(euler[0], euler[1], yaw-3.14152926)
    pose.pose.orientation.x =quaternion2[0] 
    pose.pose.orientation.y =quaternion2[1] 
    pose.pose.orientation.z =quaternion2[2] 
    pose.pose.orientation.w =quaternion2[3] 
    #pose.pose.orientation = odom.pose.pose.orientation
    pub_pose.publish(pose)
    # rospy.loginfo("%s , %s , %s ", pose.pose.position.x, pose.pose.position.y, yaw_print)
    # print("x:",pose.pose.position.x, "y:", pose.pose.position.y, "yaw:" ,yaw_print)

def listener():
    rospy.init_node('transform_to_utm')
    rospy.Subscriber('/odometry/filtered', Odometry, ekf_odom_callback)
    rospy.Subscriber('/gps/fix_velocity', TwistWithCovarianceStamped, velocity_callback)
    rospy.spin()

if __name__=='__main__':
    
    listener()
