#!/usr/bin/env python3
import socket
import rospy
from std_msgs.msg import Float32
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import string
from std_msgs.msg import String
import tf

def decode_yaw(byte1, byte2, byte3):
    """
    Decode a 3-byte yaw value following the RION protocol.
    """
    sign = -1 if byte1 >= 0x10 else 1  # Determine if negative
    value = ((byte1 & 0x0F) * 256 + byte2) + (byte3 / 100.0)
    return sign * value
# def decode_yaw(byte1, byte2, byte3):
#     """
#     Decode a 3-byte yaw value following the RION protocol.
#     """
#     sign = -1 if byte1 >= 0x10 else 1  # Determine if negative
#     value = ((byte1 & 0x0F) * 256 + byte2) + (byte3 / 100.0)
#     yaw = sign * value

#     # Ensure yaw is in [0, 360] range
#     yaw = yaw % 360  # Wrap the yaw value within 0-360°
#     return yaw
# def decode_yaw(byte1, byte2, byte3):
#     """
#     Decode a 3-byte yaw value following the RION protocol.
#     - byte1: Sign and high nibble (0x11 for negative, 0x01 for positive)
#     - byte2, byte3: Yaw angle in hundredths of a degree
#     """
#     sign = -1 if byte1 == 0x11 else  1  # Determine sign
#     raw_value = byte2 + (byte3/100)  # Combine two bytes for yaw
#     yaw = sign * (raw_value)   # Convert to degrees

#     # Ensure yaw stays within -180 to +180 for correct angle representation
#     # yaw = ((yaw + 180) % 360) - 180  
#     return yaw

def euler_to_quaternion(roll, pitch, yaw):
    """Convert roll, pitch, yaw (in radians) to quaternion."""
    return tf.transformations.quaternion_from_euler(roll, pitch, yaw)

def decode_yaw_from_string(yaw_str):
    """
    Decode a 6-character yaw string.
    - First character: 1 = Negative, 0 = Positive
    - Next five characters: Yaw value in hundredths of a degree
    """
    if len(yaw_str) != 6 or not yaw_str.isdigit():
        raise ValueError("Invalid yaw string format")

    # Extract sign (1st character)
    sign = -1 if yaw_str[0] == '1' else 1  

    # Extract the numeric yaw value (last 5 characters)
    raw_value = int(yaw_str[1:])  # Convert remaining digits to integer

    # Convert to degrees
    yaw = sign * (raw_value / 100.0)

    # Ensure yaw stays within -180° to +180°
    yaw = ((yaw + 180) % 360) - 180  

    return yaw
def decode_pitch_from_string(pitch_str):
    """
    Decode a 6-character yaw string.
    - First character: 1 = Negative, 0 = Positive
    - Next five characters: Yaw value in hundredths of a degree
    """
    if len(pitch_str) != 6 or not pitch_str.isdigit():
        raise ValueError("Invalid yaw string format")

    # Extract sign (1st character)
    sign = -1 if pitch_str[0] == '1' else 1  

    # Extract the numeric yaw value (last 5 characters)
    raw_value = int(pitch_str[1:])  # Convert remaining digits to integer
    # Convert to degrees
    pitch = sign * (raw_value / 100.0)

    # Ensure yaw stays within -180° to +180°
    pitch = ((pitch + 180) % 360) - 180  

    return pitch

def decode_roll_from_string(roll_str):
    """
    Decode a 6-character yaw string.
    - First character: 1 = Negative, 0 = Positive
    - Next five characters: Yaw value in hundredths of a degree
    """
    if len(roll_str) != 6 or not roll_str.isdigit():
        raise ValueError("Invalid yaw string format")

    # Extract sign (1st character)
    sign = -1 if roll_str[0] == '1' else 1  

    # Extract the numeric yaw value (last 5 characters)
    raw_value = int(roll_str[1:])  # Convert remaining digits to integer

    # Convert to degrees
    roll = sign * (raw_value / 100.0)

    # Ensure yaw stays within -180° to +180°
    roll = ((roll + 180) % 360) - 180  

    return roll
# def decode_yaw_from_int(int_list):
#     """
#     Decode a 6-character yaw string.
#     - First character: 1 = Negative, 0 = Positive
#     - Next five characters: Yaw value in hundredths of a degree
#     """
#     if len(int_list) != 6 or not int_list.isdigit():
#         raise ValueError("Invalid yaw  format")

#     # Extract sign (1st character)
#     sign = -1 if int_list[0] == '1' else 1  

#     # Extract the numeric yaw value (last 5 characters)
#     # raw_value = int(yaw_str[1:])  # Convert remaining digits to integer

#     # Convert to degrees
#     yaw = sign * (int_list / 100.0)

#     # Ensure yaw stays within -180° to +180°
#     yaw = ((yaw + 180) % 360) - 180  

#     return yaw
def extract_yaw_from_rx(rx_line):
    """
    Extract yaw value from an RX message line.
    """
    try:
        # Convert hex string into list of integers
        rx_bytes = [int(rx_line[i:i+2], 16) for i in range(0, len(rx_line), 2)]
        
        if len(rx_bytes) < 13:
            rospy.logwarn("Invalid RX message length")
            return None
        
        # yaw_bytes = rx_bytes[10:13]
        # yaw_bytes2 = int[rx_line[20:26]]
        yaw_chars = [rx_line[i:i+2] for i in range(20, 26, 2)]  # Get six 2-character chunks
        yaw_str = "".join(yaw_chars)
        yaw_int = int(yaw_str)

        rospy.loginfo(f"yaw_bytes_raw: {yaw_str}")
        # rospy.loginfo(f"type:  {type(yaw_bytes2)}")

        # yaw_value = decode_yaw(*yaw_bytes)
        yaw_value = decode_yaw_from_string(yaw_str)
        # yaw_value = decode_yaw_from_int(*yaw_int)


        return yaw_value
    except Exception as e:
        rospy.logerr(f"Error extracting yaw: {e}")
        return None
def extract_pitch_from_rx(rx_line):
    """
    Extract yaw value from an RX message line.
    """
    try:
        # Convert hex string into list of integers
        rx_bytes = [int(rx_line[i:i+2], 16) for i in range(0, len(rx_line), 2)]
        
        if len(rx_bytes) < 13:
            rospy.logwarn("Invalid RX message length")
            return None
        
        # yaw_bytes = rx_bytes[10:13]
        # yaw_bytes2 = int[rx_line[20:26]]
        pitch_chars = [rx_line[i:i+2] for i in range(14, 20, 2)]  # Get six 2-character chunks
        pitch_str = "".join(pitch_chars)
        pitch_int = int(pitch_str)

        rospy.loginfo(f"pitch_bytes_raw: {pitch_str}")
        # rospy.loginfo(f"type:  {type(yaw_bytes2)}")

        # yaw_value = decode_yaw(*yaw_bytes)
        pitch_value = decode_pitch_from_string(pitch_str)
        # yaw_value = decode_yaw_from_int(*yaw_int)


        return pitch_value
    except Exception as e:
        rospy.logerr(f"Error extracting pitch: {e}")
        return None
    

def extract_roll_from_rx(rx_line):
    """
    Extract yaw value from an RX message line.
    """
    try:
        # Convert hex string into list of integers
        rx_bytes = [int(rx_line[i:i+2], 16) for i in range(0, len(rx_line), 2)]
        
        if len(rx_bytes) < 13:
            rospy.logwarn("Invalid RX message length")
            return None
        
        # yaw_bytes = rx_bytes[10:13]
        # yaw_bytes2 = int[rx_line[20:26]]
        roll_chars = [rx_line[i:i+2] for i in range(8, 14, 2)]  # Get six 2-character chunks
        roll_str = "".join(roll_chars)
        roll_int = int(roll_str)

        rospy.loginfo(f"roll_bytes_raw: {roll_str}")
        # rospy.loginfo(f"type:  {type(yaw_bytes2)}")

        # yaw_value = decode_yaw(*yaw_bytes)
        roll_value = decode_roll_from_string(roll_str)
        # yaw_value = decode_yaw_from_int(*yaw_int)


        return roll_value
    except Exception as e:
        rospy.logerr(f"Error extracting roll: {e}")
        return None
    

def imu_listener():
    rospy.init_node('imu_yaw_publisher', anonymous=False)
    yaw_pub = rospy.Publisher('imu_yaw', Float32, queue_size=10)
    imu_pub = rospy.Publisher('/ML/Imu/Data', Imu, queue_size=10)
    raw_pub =rospy.Publisher('raw/hex',String,queue_size=10)
    pitch_pub =rospy.Publisher('imu_pitch', Float32, queue_size=10)
    roll_pub = rospy.Publisher('imu_roll', Float32, queue_size=10)
    
    HOST = "192.168.1.201"  # IMU device IP
    PORT = 4196  # IMU device port
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        rospy.loginfo("Connected to IMU")
        
        while not rospy.is_shutdown():
            data = s.recv(1024)  # Read raw data
            if not data:
                break
            hex_data = data.hex().upper()
            rospy.loginfo(f"Received: {hex_data}")
            raw_pub.publish(hex_data)

            yaw_value = extract_yaw_from_rx(hex_data)
            #pitch_value = extract_pitch_from_rx(hex_data)
            #roll_value =extract_roll_from_rx(hex_data)
            roll_value = extract_pitch_from_rx(hex_data)
            pitch_value =extract_roll_from_rx(hex_data)
            
            
            if yaw_value is not None and pitch_value is not None and roll_value is not None:
                yaw_pub.publish(yaw_value)
                roll_pub.publish(roll_value)
                pitch_pub.publish(pitch_value)
                rospy.loginfo(f"Yaw: {yaw_value}°")
                rospy.loginfo(f"pitch: {pitch_value}°")
                rospy.loginfo(f"roll: {roll_value}°")
                
    

                # Convert degrees to radians
                roll_rad = math.radians(roll_value)
                pitch_rad = math.radians(pitch_value)
                yaw_rad = math.radians(yaw_value)

                # Create quaternion from RPY
                q = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

                # Build IMU message
                imu_msg = Imu()
                imu_msg.header = Header()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = "imu_link"

                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]

                # Fill angular velocity (set to zero or your actual values)
                imu_msg.angular_velocity.x = 0.0
                imu_msg.angular_velocity.y = 0.0
                imu_msg.angular_velocity.z = 0.0

                # Fill linear acceleration (optional or zero)
                imu_msg.linear_acceleration.x = 0.0
                imu_msg.linear_acceleration.y = 0.0
                imu_msg.linear_acceleration.z = 0.0

                # Covariances
                imu_msg.orientation_covariance = [0.01, 0, 0,
                                                0, 0.01, 0,
                                                0, 0, 0.01]

                imu_msg.angular_velocity_covariance = [-1.0] * 9
                imu_msg.linear_acceleration_covariance = [-1.0] * 9



                # imu_msg = Imu()
                # imu_msg.header = Header()
                # imu_msg.header.stamp = rospy.Time.now()
                # imu_msg.header.frame_id = "imu_link"

                # # Convert yaw to quaternion
                # yaw_rad = math.radians(yaw_value)
                # imu_msg.orientation.z = math.sin(yaw_rad / 2)
                # imu_msg.orientation.w = math.cos(yaw_rad / 2)

                # # Fill angular velocity (if available, otherwise set to zero)
                # imu_msg.angular_velocity.x = 0.0
                # imu_msg.angular_velocity.y = 0.0
                # imu_msg.angular_velocity.z = 0.0  # Update if angular rate is available

                # # Covariance matrices
                # imu_msg.orientation_covariance = [0.0] * 9
                # imu_msg.orientation_covariance[8] = 0.01  # Low uncertainty for yaw
                # imu_msg.angular_velocity_covariance = [-1.0] * 9  # Unknown angular velocity

                  

                imu_pub.publish(imu_msg)

                rospy.loginfo(f"Published IMU message with yaw: {yaw_value}°")

if __name__ == "__main__":
    try:
        imu_listener()
    except rospy.ROSInterruptException:
        pass

