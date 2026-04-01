#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
import subprocess
import sys, select 
import string

class LiDAR_Measurements(Node):
    def __init__(self):
        super().__init__("lidar_readings_extractor_node")
        qos = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, reliability = QoSReliabilityPolicy.RELIABLE, depth=50)

        self.scan_publisher_tugbot_1 = self.create_publisher(LaserScan, 'front_lidar_scan_tugbot_1', qos)
        self.scan_publisher_tugbot_2 = self.create_publisher(LaserScan, 'front_lidar_scan_tugbot_2', qos)
        self.scan_publisher_tugbot_3 = self.create_publisher(LaserScan, 'front_lidar_scan_tugbot_3', qos)
        command_lidar_tugbot_1 = ["gz", "topic", "--echo", "--topic", "/world/world_demo/model/tugbot_1/link/scan_front/sensor/scan_front/scan"]
        command_lidar_tugbot_2 = ["gz", "topic", "--echo", "--topic", "/world/world_demo/model/tugbot_2/link/scan_front/sensor/scan_front/scan"]
        command_lidar_tugbot_3 = ["gz", "topic", "--echo", "--topic", "/world/world_demo/model/tugbot_3/link/scan_front/sensor/scan_front/scan"]
        
        self.process_lidar_tugbot_1 = subprocess.Popen(command_lidar_tugbot_1, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.process_lidar_tugbot_2 = subprocess.Popen(command_lidar_tugbot_2, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.process_lidar_tugbot_3 = subprocess.Popen(command_lidar_tugbot_3, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

        self.timer = self.create_timer(1/100, self.timer_callback)
    
    def timer_callback(self):
        
        #Variables to utilize lidar on tugbot 1.
        all_distances_lidar_tugbot_1 = []
        phi_lidar_tugbot_1  = []
        h=0
        while select.select([self.process_lidar_tugbot_1.stdout], [], [], 0)[0]:
            line_lidar_tugbot_1  = self.process_lidar_tugbot_1.stdout.readline().decode().strip()
            if line_lidar_tugbot_1.startswith("ranges:"):
        
                for s in line_lidar_tugbot_1.split(":",1)[1].split():
                    distance_lidar_tugbot_1 = float(s)
                    bearing_lidar_tugbot_1 = (h*0.0043698) -1.47
                    phi_lidar_tugbot_1.append(bearing_lidar_tugbot_1)
                    all_distances_lidar_tugbot_1.append(distance_lidar_tugbot_1)
                    h=h+1

                if len(all_distances_lidar_tugbot_1) >= 650:
                    laser_message_lidar_tugbot_1 = LaserScan()
                    laser_message_lidar_tugbot_1.header.frame_id = 'lidar_link'
                    laser_message_lidar_tugbot_1.angle_min = -1.47043991089
                    laser_message_lidar_tugbot_1.angle_max = 1.47043991089
                    laser_message_lidar_tugbot_1.angle_increment = 0.0043698065702526 #(laser_message.angle_max - laser_message.angle_min) / len(all_distances)
                    laser_message_lidar_tugbot_1.range_min = 0.01
                    laser_message_lidar_tugbot_1.range_max = 5.0
                    laser_message_lidar_tugbot_1.ranges = all_distances_lidar_tugbot_1
                    self.scan_publisher_tugbot_1.publish(laser_message_lidar_tugbot_1)
                    phi_lidar_tugbot_1.clear()
                    all_distances_lidar_tugbot_1.clear()
                    h=0

        #Variables to utilize lidar on tugbot 2.
        all_distances_lidar_tugbot_2 = []
        phi_lidar_tugbot_2  = []
        p=0
        while select.select([self.process_lidar_tugbot_2.stdout], [], [], 0)[0]:
            line_lidar_tugbot_2  = self.process_lidar_tugbot_2.stdout.readline().decode().strip()
            if line_lidar_tugbot_2.startswith("ranges:"):
        
                for g in line_lidar_tugbot_2.split(":",1)[1].split():
                    distance_lidar_tugbot_2 = float(g)
                    bearing_lidar_tugbot_2 = (p*0.0043698) -1.47
                    phi_lidar_tugbot_2.append(bearing_lidar_tugbot_2)
                    all_distances_lidar_tugbot_2.append(distance_lidar_tugbot_2)
                    p=p+1

                if len(all_distances_lidar_tugbot_2) >= 650:
                    laser_message_lidar_tugbot_2 = LaserScan()
                    laser_message_lidar_tugbot_2.header.frame_id = 'lidar_link'
                    laser_message_lidar_tugbot_2.angle_min = -1.47043991089
                    laser_message_lidar_tugbot_2.angle_max = 1.47043991089
                    laser_message_lidar_tugbot_2.angle_increment = 0.0043698065702526 #(laser_message.angle_max - laser_message.angle_min) / len(all_distances)
                    laser_message_lidar_tugbot_2.range_min = 0.01
                    laser_message_lidar_tugbot_2.range_max = 5.0
                    laser_message_lidar_tugbot_2.ranges = all_distances_lidar_tugbot_2
                    self.scan_publisher_tugbot_2.publish(laser_message_lidar_tugbot_2)
                    phi_lidar_tugbot_2.clear()
                    all_distances_lidar_tugbot_2.clear()
                    p=0
        
        #Variables to utilize lidar on tugbot 3.
        all_distances = []
        phi = []
        i=0
        while select.select([self.process_lidar_tugbot_3.stdout], [], [], 0)[0]:
            line = self.process_lidar_tugbot_3.stdout.readline().decode().strip()
            if line.startswith("ranges:"):
        
                for v in line.split(":",1)[1].split():
                    distance = float(v)
                    bearing = (i*0.0043698) -1.47
                    phi.append(bearing)
                    all_distances.append(distance)
                    #if all_distances[i] >= 0.1 and all_distances[i] <= 2.0:
                    #    print(all_distances[i])
                    #    print(i)
                    i=i+1
                if len(all_distances) >= 650:
                    laser_message = LaserScan()
                    laser_message.header.frame_id = 'lidar_link'
                    laser_message.angle_min = -1.47043991089
                    laser_message.angle_max = 1.47043991089
                    laser_message.angle_increment = 0.0043698065702526 #(laser_message.angle_max - laser_message.angle_min) / len(all_distances)
                    laser_message.range_min = 0.01
                    laser_message.range_max = 5.0
                    laser_message.ranges = all_distances
                    #print(all_distances[300])
                    #print(phi[300])
                    #if len(phi) > 380:
                    #    print(phi[379])
                    #    print("d")
                    #    print(all_distances[379])
                    self.scan_publisher_tugbot_3.publish(laser_message)
                    phi.clear()
                    all_distances.clear()
                    i=0
        
def main (args=None):
    rclpy.init(args=args)
    node = LiDAR_Measurements()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()