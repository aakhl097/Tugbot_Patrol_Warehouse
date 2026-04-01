#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import string
import math
import threading

class Tugbot_2_Controller(Node):
    def __init__(self):
        super().__init__("tugbot_1_controller")
        qos = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, reliability = QoSReliabilityPolicy.RELIABLE, depth=10)
        #front scan
        self.lidar_readings = LaserScan()
        self.coordinates_tugbot_2 = Pose()
        self.bearings_tugbot_2 = Pose()
        self.commanded_velocities_tugbot_2 = Twist()
        self.velocity_publisher_tugbot_2 = self.create_publisher(Twist, 'model/tugbot_2/cmd_vel', qos)
        self.position_listener_tugbot_2 = self.create_subscription(Pose, 'cartesian_position_tugbot_2', self.position_callback_tugbot_2, qos)
        self.orientation_listener_tugbot_2 = self.create_subscription(Pose, 'orientation_tugbot_2', self.orientation_callback_tugbot_2, qos)
        self.lidar_measurements = self.create_subscription(LaserScan, 'front_lidar_scan_tugbot_2', self.lidar_callback_tugbot_2, qos)
        self.timer_tugbot_2 = self.create_timer(1/30, self.timer_callback_tugbot_2)
        self.timer = self.create_timer(1/50, self.timer_callback)
        self.rotate_left_tugbot_2 = True
        self.drive_straight_tugbot_2 = False
        self.drive_straight_again_tugbot_2 = False
        self.following_tugbot_1 = False
        self.collision_alert = False

    def position_callback_tugbot_2(self, msg_tugbot_2):
        self.coordinates_tugbot_2 = msg_tugbot_2

    def orientation_callback_tugbot_2(self, msg_tugbot_2):
        self.bearings_tugbot_2 = msg_tugbot_2

    def lidar_callback_tugbot_2(self, lidar_msg_tugbot_2):
        self.lidar_readings = lidar_msg_tugbot_2

    def collision_detection(self):
        all_distances = []
        if self.lidar_readings is not None:
            all_distances = self.lidar_readings.ranges
            if len(all_distances) >= 650: 

                if (all_distances[320] >= 0.1 and all_distances[320] <= 2.5):
                    self.collision_alert = True
                elif (all_distances[300] >= 0.1 and all_distances[300] <= 2.5):
                    self.collision_alert = True
                elif (all_distances[370] >= 0.1 and all_distances[370] <= 2.5):
                    self.collision_alert = True
                elif (all_distances[350] >= 0.1 and all_distances[350] <= 2.5):
                    self.collision_alert = True
                else:
                    self.collision_alert = False

    def timer_callback(self):
        thread_lidar = threading.Thread(target = self.collision_detection)
        thread_lidar.start()
    
    def timer_callback_tugbot_2(self):

        #thread_lidar = threading.Thread(target = self.collision_detection)
        #thread_lidar.start()
        
        if self.rotate_left_tugbot_2 == True:
            self.commanded_velocities_tugbot_2.angular.z = 0.3
            self.commanded_velocities_tugbot_2.linear.x = 0.0

        #x = -5.3162, y = 0.4
        if ((self.bearings_tugbot_2.orientation.z >= 1.2) and (self.bearings_tugbot_2.orientation.z <= 1.35)):
            self.commanded_velocities_tugbot_2.angular.z = 0.0
            self.commanded_velocities_tugbot_2.linear.x = 1.2 #drive diagonally.
            self.rotate_left_tugbot_2 = False
            self.drive_straight_tugbot_2 = True

        if self.drive_straight_tugbot_2 == True:
            if ((self.coordinates_tugbot_2.position.x >=-6.5) and (self.coordinates_tugbot_2.position.x <=-4.7) and (self.coordinates_tugbot_2.position.y >=-0.1) and (self.coordinates_tugbot_2.position.y <= 0.5)):
                self.commanded_velocities_tugbot_2.linear.x = 0.0
                self.commanded_velocities_tugbot_2.angular.z = -0.2 #-0.2
                self.drive_straight_tugbot_2 = False
                self.drive_straight_again_tugbot_2 = True
        
        if self.drive_straight_again_tugbot_2 == True:
            if ((self.bearings_tugbot_2.orientation.z <= 0.03) and (self.bearings_tugbot_2.orientation.z >= -0.01)):
                self.commanded_velocities_tugbot_2.angular.z = 0.0
                self.commanded_velocities_tugbot_2.linear.x = 2.0 
                self.drive_straight_again_tugbot_2 = False
                self.following_tugbot_1 = True

        if self.following_tugbot_1 == True:
            if self.collision_alert == True:
                self.commanded_velocities_tugbot_2.angular.z = 0.0
                self.commanded_velocities_tugbot_2.linear.x = 0.0
            else:
                self.commanded_velocities_tugbot_2.angular.z = 0.0
                self.commanded_velocities_tugbot_2.linear.x = 2.0
          
        self.velocity_publisher_tugbot_2.publish(self.commanded_velocities_tugbot_2)  #-3.93 0.3

def main(args=None):
    rclpy.init(args=args)
    node = Tugbot_2_Controller()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
