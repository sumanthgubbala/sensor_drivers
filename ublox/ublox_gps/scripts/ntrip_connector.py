#!/usr/bin/env python3
import socket
import base64
import serial
import threading
# from pyrtcm import RTCMReader
import rospy
from std_msgs.msg import String 
import queue

class ntrip_communication:
    def __init__(self):
    # NTRIP Caster Configuration
        self.caster_host = '103.206.29.4'
        self.caster_port = 2105
        self.mountpoint = 'IVRS'
        self.username = 'p.arjun'
        self.password = 'cors@2022'

    # GNSS receiver configuration
        # self.gnss_port = '/dev/ttyACM0'
        # self.gnss_baud = 115200 # Use the correct baudrate for your GNSS receiver
        self.header = b""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_file = self.sock.makefile('rb')
        # self.reader = RTCMReader(self.sock_file)
        self.usb_ublox_port ='/dev/ttyUSB0'
        self.usb_baud = 115200
        self.request = (
            f"GET /{self.mountpoint} HTTP/1.0\r\n"
            f"User-Agent: NTRIP pyClient/1.0\r\n"
            f"Authorization: Basic {base64.b64encode(f'{self.username}:{self.password}'.encode()).decode()}\r\n"
            "\r\n"
        )
        
    # Connect to the NTRIP caster
        self.sock.connect((self.caster_host, self.caster_port))
        self.sock.sendall(self.request.encode())
        self.usb_serial =serial.Serial(self.usb_ublox_port,self.usb_baud,timeout=1)

        self.ntrip_authentication()

        # self.ntrip_authentication()

      

        # self.data =self.sock.recv(4096)
        self.latest_gga = None
        self.gga_ready = False
        # self.socket_thread = threading.Thread(target=self.socket_callback)
        # self.socket_thread.daemon = True  # Daemon thread will exit with the node
        # self.socket_thread.start()
        # self.gga_queue = queue.Queue()

    def ntrip_authentication(self):
    # Receive and skip HTTP headers

        while b"\r\n\r\n" not in self.header:
            self.header += self.sock.recv(4096)
            print(self.header)
        if b"ICY 200 OK" not in self.header:
            print("Failed to connect to mountpoint")
            self.sock.close()
            exit()
        print("Connected to caster, waiting for correction data...")
        # self.gnss_data_callback()
        # self.raw_gga()
        # self.socket_callback()

    def gnss_data_callback(self,msg):
        # line = msg.readline().decode(errors="ignore").strip()
        
            print(f"data: {msg}")
            line = msg.data.strip()
            print(f"line: {line}")

        # if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
            self.latest_gga = line + "\r\n"
            self.gga_ready = True
        # self.socket_callback()       
            if self.gga_ready and self.latest_gga:        
                try:
                    
                    print(f"latest gga{self.latest_gga}")

                        # gga = line + "\r\n"
                    self.sock.sendall(self.latest_gga.encode())
                                    # print(f"[GGA] Sent: {gga.strip()}")
                    print(f"[GGA] Sent: {self.latest_gga}")
                                # hex_data = self.data.decode
                    print(f"socket data: {self.sock.recv(1024)}")
                    self.usb_dumping()

                    self.gga_ready = False 

                                # print(f"hex data: {hex_data}")


                    try:
                                        # send_gga_loop()
                                    # for raw, parsed in self.reader:
                                    #     print(f"[RTCM] Type: {parsed.identity}, Length: {parsed.length}")
                        self.usb_dumping()

                    except KeyboardInterrupt:
                            print("Stopped by user.")
                    finally:
                            self.sock.close()
                                        # gnss_serial.close()
                except Exception as e:
                  if(e=="Attempting to use a port that is not open"):
                        print(f"port status:- {self.gnss_serial.is_open}")
                           
                  else:
                    print(f"[GGA Error] {e}")
                            # gnss_serial.close()
                    exit
                except KeyboardInterrupt:
                            print("Stopped by user.") 

    # def socket_callback(self):
    #     while  not rospy.is_shutdown:
                
    #         if self.gga_ready and self.latest_gga:        
    #             try:
                    
    #                 print(f"latest gga{self.latest_gga}")

    #                     # gga = line + "\r\n"
    #                 self.sock.sendall(self.latest_gga.encode())
    #                                 # print(f"[GGA] Sent: {gga.strip()}")
    #                 print(f"[GGA] Sent: {self.latest_gga}")
    #                             # hex_data = self.data.decode
    #                 print(f"socket data: {self.sock.recv(1024)}")
    #                 self.usb_dumping()

    #                 self.gga_ready = False 

    #                             # print(f"hex data: {hex_data}")


    #                 try:
    #                                     # send_gga_loop()
    #                                 # for raw, parsed in self.reader:
    #                                 #     print(f"[RTCM] Type: {parsed.identity}, Length: {parsed.length}")
    #                     self.usb_dumping()

    #                 except KeyboardInterrupt:
    #                         print("Stopped by user.")
    #                 finally:
    #                         self.sock.close()
    #                                     # gnss_serial.close()
    #             except Exception as e:
    #               if(e=="Attempting to use a port that is not open"):
    #                     print(f"port status:- {self.gnss_serial.is_open}")
                            
    #               else:
    #                 print(f"[GGA Error] {e}")
    #                         # gnss_serial.close()
    #                 exit
    def usb_dumping(self):
         self.usb_serial.write(self.sock.recv(1024))
        #  self.gnss_serial.write(self.sock.recv(1024))

         
    def main(self):
        rospy.init_node('ntrip_connector',anonymous=False)
        rospy.Subscriber("/gps/raw_data_stream",String,self.gnss_data_callback)
        print("subscriber started.........")
        
        # Connect to GNSS serial for GGA
        # self.gnss_serial = serial.Serial(self.gnss_port, self.gnss_baud, timeout=1)
        # self.raw_gga()

    # NTRIP v1 Request
        
if __name__=="__main__":
    
      ntrip = ntrip_communication() 
      ntrip.main()
      rospy.spin()
      

    
    # Start thread to send GGA
# gga_thread = threading.Thread(target=send_gga_loop, daemon=True)
# gga_thread.start()

# Read RTCM data




# try:
#     while True:
#         data = sock.recv(1024)
#         if not data:
#             print("No more correction data.")
#             break
#         for byte in data:
#             msg = reader.read(bytes([byte]))
#             if msg is not None:
#                 raw, parsed = msg
#                 print(f"[RTCM] Type: {parsed.identity}, Length: {parsed.length}")
# except KeyboardInterrupt:
#     print("Stopped by user.")
# finally:
#     sock.close()
#     gnss_serial.close()

#     sock.close()
    # gnss_serial.close()

# import rospy
# from sensor_msgs.msg import NavSatFix, NavSatStatus
# import socket
# import serial
# import base64
# from pyrtklib import *
# from pyrtklib.rtklib import RTK, rtkinit, rtkfree, input_rtcm3

# class RTKGNSSNode:
#     def __init__(self):
#         # ROS publisher
#         self.pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)

#         # Parameters (can also use ROS params)
#         self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
#         self.baudrate = rospy.get_param('~baudrate', 115200)
#         self.ntrip_host = rospy.get_param('~ntrip_host', '103.206.29.4')
#         self.ntrip_port = rospy.get_param('~ntrip_port', 2105)
#         self.mountpoint = rospy.get_param('~mountpoint', 'IVRS')
#         self.username = rospy.get_param('~username', 'p.arjun')
#         self.password = rospy.get_param('~password', 'cors@2022')

#         # Setup serial
#         self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)

#         # Setup RTKLib
#         self.rtk = RTK()
#         rtkinit(self.rtk)

#         # Connect to NTRIP
#         self.ntrip_socket = self.connect_ntrip()

#         rospy.loginfo("RTK GNSS node initialized")

#     def connect_ntrip(self):
#         creds = f"{self.username}:{self.password}"
#         auth = base64.b64encode(creds.encode()).decode()
#         headers = (
#             f"GET /{self.mountpoint} HTTP/1.0\r\n"
#             f"User-Agent: NTRIP pyRTKLib/1.0\r\n"
#             f"Authorization: Basic {auth}\r\n"
#             f"\r\n"
#         )

#         s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         s.connect((self.ntrip_host, self.ntrip_port))
#         s.sendall(headers.encode())

#         # Read past HTTP headers
#         response = b""
#         while b"\r\n\r\n" not in response:
#             response += s.recv(1)

#         rospy.loginfo("Connected to NTRIP caster")
#         return s

#     def spin(self):
#         rate = rospy.Rate(10)  # 10 Hz

#         while not rospy.is_shutdown():
#             # Receive RTCM data and feed to RTKLib
#             try:
#                 rtcm_data = self.ntrip_socket.recv(1024)
#                 if rtcm_data:
#                     input_rtcm3(self.rtk, rtcm_data)
#                     # Also forward RTCM data to GNSS receiver
#                     self.ser.write(rtcm_data)
#             except socket.timeout:
#                 pass
#             except Exception as e:
#                 rospy.logwarn(f"NTRIP socket error: {e}")

#             # Optionally read GNSS data from serial and feed to RTKLib
#             # raw_gnss_data = self.ser.read(self.ser.in_waiting or 1)
#             # input_raw(self.rtk, raw_gnss_data)  # If implemented

#             # Check solution and publish
#             if self.rtk.sol.stat >= 1:
#                 fix = NavSatFix()
#                 fix.header.stamp = rospy.Time.now()
#                 fix.header.frame_id = "gps"

#                 # lat, lon, height in degrees/meters
#                 fix.latitude = self.rtk.sol.rr[0]
#                 fix.longitude = self.rtk.sol.rr[1]
#                 fix.altitude = self.rtk.sol.rr[2]

#                 # Status
#                 fix.status.status = NavSatStatus.STATUS_FIX if self.rtk.sol.stat >= 1 else NavSatStatus.STATUS_NO_FIX
#                 fix.status.service = NavSatStatus.SERVICE_GPS

#                 # Covariance unknown
#                 fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

#                 self.pub.publish(fix)
#                 rospy.loginfo_throttle(1, f"Published GPS fix: lat={fix.latitude:.7f}, lon={fix.longitude:.7f}")

#             rate.sleep()

#     def shutdown(self):
#         rtkfree(self.rtk)
#         self.ser.close()
#         self.ntrip_socket.close()
#         rospy.loginfo("RTK GNSS node shutdown")

# if __name__ == "__main__":
#     rospy.init_node('rtk_gnss_node')
#     node = RTKGNSSNode()
#     try:
#         node.spin()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         node.shutdown()


# # import base64 
# # from pyrtklib.rtklib import RTK, nav_t, obs_t, sol_t, rtkinit, rtkfree, input_rtcm3
# # serial_port = "/dev/ttyACM0"
# # baudrate = 115200
# # ntrip_host ="103.206.29.4"


# # ntrip_port=2105
# # mountpoint="IVRS"
# # username="p.arjun"
# # password= "cors@2022"
# # print(f"Connecting to {ntrip_host}:{ntrip_port} for mountpoint {mountpoint}")
# # print(f"Forwarding RTCM data to GNSS module on {serial_port} at {baudrate} bps")

# # # Create and run NTRIP client
# # ntrip_client = NTRIPClient(
# #     server=ntrip_host,
# #     port=ntrip_port,
# #     mountpoint=mountpoint,
# #     user=username,
# #     password=password,
# #     serialport=serial_port,
# #     baudrate=baudrate
# # )

# # try:
# #     ntrip_client.run()  # This will block and continuously stream data
# # except KeyboardInterrupt:
# #     print("Stopped by user")
# # finally:
# #     ntrip_client.stop()
