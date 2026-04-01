#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import string
import math
import threading

class Tugbot_3_Controller(Node):
    def __init__(self):
        super().__init__("tugbot_1_controller")
        qos = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, reliability = QoSReliabilityPolicy.RELIABLE, depth=10)

        self.lidar_readings = LaserScan()
        self.coordinates_tugbot_3 = Pose()
        self.bearings_tugbot_3 = Pose()
        self.commanded_velocities_tugbot_3 = Twist()
        self.velocity_publisher_tugbot_3 = self.create_publisher(Twist, 'model/tugbot_3/cmd_vel', qos)
        self.position_listener_tugbot_3 = self.create_subscription(Pose, 'cartesian_position_tugbot_3', self.position_callback_tugbot_3, qos)
        self.orientation_listener_tugbot_3 = self.create_subscription(Pose, 'orientation_tugbot_3', self.orientation_callback_tugbot_3, qos)
        self.lidar_measurements = self.create_subscription(LaserScan, 'front_lidar_scan_tugbot_3', self.lidar_callback_tugbot_3, qos)
        self.timer_tugbot_3 = self.create_timer(1/30, self.timer_callback_tugbot_3)
        self.timer = self.create_timer(1/50, self.timer_callback)

        self.rotate_right_tugbot_3 = True
        self.drive_straight_tugbot_3 = False
        self.drive_straight_again_tugbot_3 = False

        self.following_tugbot_1 = False
        self.collision_alert = False 
        self.distance_ray_320 = 0.0
        self.distance_ray_390 = 0.0
        self.k = 0

    def position_callback_tugbot_3(self, msg_tugbot_3):
        self.coordinates_tugbot_3 = msg_tugbot_3

    def orientation_callback_tugbot_3(self, msg_tugbot_3):
        self.bearings_tugbot_3 = msg_tugbot_3

    def lidar_callback_tugbot_3(self, lidar_msg_tugbot_3):
        self.lidar_readings = lidar_msg_tugbot_3
    
    def collision_detection(self):
        all_distances = []
        if self.lidar_readings is not None:
            all_distances = self.lidar_readings.ranges
            if len(all_distances) >= 650: 
                
            #NOTES: For loops create delays. Snippet below is a good alternative to for loops.
            #Other delays are created by QoS and depth (queue) of messages.
            #i = 0
            #for i in range(len(all_distances)):
            #    if ((i == 320) or (i==300) or (i == 350) or (i==370) or (i==280) or (i==400) or (i==250)): #Select 6 rays from lidar corresponding to center area.
                    #if (all_distances[i] >= 0.1 and all_distances[i] <= 3.0):
                    #    self.collision_alert = True
                        #print("Warning")
                        #print("\n")
                        #print(i)
                        #print(all_distances[i])

                if (all_distances[320] >= 0.1 and all_distances[320] <= 2.5):
                    self.collision_alert = True
                elif (all_distances[370] >= 0.1 and all_distances[370] <= 2.5):
                    self.collision_alert = True
                elif (all_distances[350] >= 0.1 and all_distances[350] <= 2.5):
                    self.collision_alert = True
                elif (all_distances[400] >= 0.1 and all_distances[400] <= 2.5):
                    self.collision_alert = True
                else:
                    self.collision_alert = False
                #i=i+1

    def timer_callback(self): 
        thread_lidar = threading.Thread(target = self.collision_detection)
        thread_lidar.start()
    
    def timer_callback_tugbot_3(self):

        #thread_lidar = threading.Thread(target = self.collision_detection)
        #thread_lidar.start()
        
        #a_distances = self.lidar_readings.ranges
        #if self.k == 320:
        #    print(a_distances[self.k])
        
        if self.rotate_right_tugbot_3 == True:
            self.commanded_velocities_tugbot_3.angular.z = -0.3
            self.commanded_velocities_tugbot_3.linear.x = 0.0

        #x = -2.0586, y = 0.126
        if ((self.bearings_tugbot_3.orientation.z >= -1.3) and (self.bearings_tugbot_3.orientation.z <= -1.1)):
            self.commanded_velocities_tugbot_3.angular.z = 0.0
            self.commanded_velocities_tugbot_3.linear.x = 1.2 #drive diagonally.
            self.rotate_right_tugbot_3 = False
            self.drive_straight_tugbot_3 = True

        if self.drive_straight_tugbot_3 == True: #y was 0.65 before.
            if ((self.coordinates_tugbot_3.position.x >=-4.0) and (self.coordinates_tugbot_3.position.x <=-3.4) and (self.coordinates_tugbot_3.position.y >=0.05) and (self.coordinates_tugbot_3.position.y <= 1.2)):
                self.commanded_velocities_tugbot_3.linear.x = 0.0
                self.commanded_velocities_tugbot_3.angular.z = 0.25
                self.drive_straight_tugbot_3 = False
                self.drive_straight_again_tugbot_3 = True
        
        if self.drive_straight_again_tugbot_3 == True:
            if ((self.bearings_tugbot_3.orientation.z >= -0.0425) and (self.bearings_tugbot_3.orientation.z <= -0.01)):
                self.commanded_velocities_tugbot_3.angular.z = 0.0
                self.commanded_velocities_tugbot_3.linear.x = 2.0 
                self.following_tugbot_1 = True
                self.drive_straight_again_tugbot_3 = False

        #self.collision_detection()

        if self.following_tugbot_1 == True:
            if self.collision_alert == True:
                self.commanded_velocities_tugbot_3.angular.z = 0.0
                self.commanded_velocities_tugbot_3.linear.x = 0.0
            else:
                self.commanded_velocities_tugbot_3.angular.z = 0.0
                self.commanded_velocities_tugbot_3.linear.x = 2.0
          
        self.velocity_publisher_tugbot_3.publish(self.commanded_velocities_tugbot_3)  #-3.93 0.3
        #print(self.bearings_tugbot_3.orientation.z)

def main(args=None):
    rclpy.init(args=args)
    node = Tugbot_3_Controller()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
