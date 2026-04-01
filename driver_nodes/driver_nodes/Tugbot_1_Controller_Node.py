#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import string
import math
import threading

class Tugbot_1_Controller(Node):
    def __init__(self):
        super().__init__("tugbot_1_controller_first_floor")
        qos = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, reliability = QoSReliabilityPolicy.RELIABLE, depth=10)

        self.lidar_readings = LaserScan()
        self.coordinates = Pose()
        self.bearings = Pose()
        self.commanded_velocities = Twist()
        self.flag = Bool()
        self.flag.data = False
        self.velocity_publisher_tugbot_1 = self.create_publisher(Twist, 'model/tugbot_1/cmd_vel', qos)
        self.flag_publisher_floor_1_completed = self.create_publisher(Bool, 'flag_floor_1_completed', qos)
        self.position_listener_tugbot_1 = self.create_subscription(Pose, 'cartesian_position_tugbot_1', self.position_callback, qos)
        self.orientation_listener_tugbot_1 = self.create_subscription(Pose, 'orientation_tugbot_1', self.orientation_callback, qos)
        self.lidar_measurements = self.create_subscription(LaserScan, 'front_lidar_scan_tugbot_1', self.lidar_callback_tugbot_1, qos)
        self.timer = self.create_timer(1/30, self.timer_callback)
        self.collision_alert = False
        self.rotate_right = True
        self.drive_straight = False
        self.drive_straight_again = False
        self.tugbot_1_leading = False

        self.rotated_left_to_elevator = False
        self.rotate_right_to_elevator = False
        self.go_to_elevator = False
        self.inside_elevator = False
        self.deactivate_lidar = False
        
        #x = 7.19, y =1.0998

    def position_callback(self, msg):
        self.coordinates = msg

    def orientation_callback(self, msg):
        self.bearings = msg

    def lidar_callback_tugbot_1(self, lidar_msg_tugbot_1):
        self.lidar_readings = lidar_msg_tugbot_1

    def collision_detection(self):
        all_distances = []
        if (self.lidar_readings is not None):
            all_distances = self.lidar_readings.ranges
            if len(all_distances) >= 650: 

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

    def timer_callback(self):

        if self.deactivate_lidar == False:
            thread_lidar = threading.Thread(target = self.collision_detection)
            thread_lidar.start()
        
        if self.rotate_right == True:
            self.commanded_velocities.angular.z = -0.1
            self.commanded_velocities.linear.x = 0.0

        if ((self.bearings.orientation.z >= -0.33) and (self.bearings.orientation.z <= -0.2)):
            self.commanded_velocities.angular.z = 0.0
            self.commanded_velocities.linear.x = 0.8 #drive diagonally.
            self.rotate_right = False
            self.drive_straight = True

        if self.drive_straight == True:
            if ((self.coordinates.position.x >=-2.93) and (self.coordinates.position.x <=-2.4) and (self.coordinates.position.y >=-3.33) and (self.coordinates.position.y <= 1.2)):
                self.commanded_velocities.linear.x = 0.0
                self.commanded_velocities.angular.z = 0.03 #0.04
                self.drive_straight = False
                self.drive_straight_again = True
        
        if self.drive_straight_again == True:
            if ((self.bearings.orientation.z >= -0.01) and (self.bearings.orientation.z <= 0.02)):
                self.commanded_velocities.angular.z = 0.0
                self.commanded_velocities.linear.x = 2.0
                self.tugbot_1_leading = True
                self.drive_straight_again = False

        if self.tugbot_1_leading == True:
            
            if self.collision_alert == True:
                self.commanded_velocities.angular.z = 0.0
                self.commanded_velocities.linear.x = 0.0
            else:
                self.commanded_velocities.angular.z = 0.0
                self.commanded_velocities.linear.x = 2.0

            if ((self.coordinates.position.x <= 18.25 and self.coordinates.position.x >= 17.0) and (self.coordinates.position.y <= 1.1 and self.coordinates.position.y >= -0.172)):
                self.commanded_velocities.linear.x = 0.0

                if self.rotated_left_to_elevator == False:
                    self.commanded_velocities.angular.z = 0.15

                if ((self.bearings.orientation.z <= 1.68) and (self.bearings.orientation.z >= 1.5)):
                    self.commanded_velocities.angular.z = 0.0
                    self.commanded_velocities.linear.x = 0.6
                    self.rotated_left_to_elevator = True

            if ((self.coordinates.position.x >= 17.5536 and self.coordinates.position.x <= 18.8) and (self.coordinates.position.y >= 4.687 and self.coordinates.position.y <= 6.2215)):
                self.commanded_velocities.linear.x = 0.0
                
                if self.rotate_right_to_elevator == False:
                    self.commanded_velocities.angular.z = -0.15
            
                if ((self.bearings.orientation.z >= -0.015) and (self.bearings.orientation.z <= 0.04)):
                    self.deactivate_lidar = True
                    self.commanded_velocities.linear.x = 0.8
                    self.commanded_velocities.angular.z = 0.0
                    self.rotate_right_to_elevator = True
                    self.go_to_elevator = True

            if self.go_to_elevator == True:
                self.collision_alert = False
                self.commanded_velocities.linear.x = 1.61 #0.61

                #new 
                if (self.bearings.orientation.z >= 0.03):
                    self.commanded_velocities.angular.z = -0.1
                elif (self.bearings.orientation.z <= -0.03):
                    self.commanded_velocities.angular.z = 0.1
                else:
                    self.commanded_velocities.angular.z = 0.0
                
                if ((self.coordinates.position.x >= 20.3668 and self.coordinates.position.x <= 20.446) and (self.coordinates.position.y >= 5.758 and self.coordinates.position.y <= 6.1)): #20.1968
                    self.inside_elevator = True
                    self.go_to_elevator = False #x >= 20.1668 and x <=20.246
                    
            if self.inside_elevator == True:
                self.commanded_velocities.angular.z = 0.0
                self.commanded_velocities.linear.x = 0.0
                self.get_logger().info("Tugbot 1 is requesting the platform to be activated.")
                if (self.bearings.orientation.z >= 0.03):
                    self.commanded_velocities.angular.z = -0.1
                elif (self.bearings.orientation.z <= -0.03):
                    self.commanded_velocities.angular.z = 0.1
                else:
                    self.commanded_velocities.angular.z = 0.0
                if (self.coordinates.position.z >= 3.0):
                    self.tugbot_1_leading = False
                    self.flag.data = True
                    self.inside_elevator = False

        # 18 < x < 19 and -0.172 < y < 1.1
        # 18.8744 < x < 18.96 and 5.1514 < y < 6.2259
        # 20.38 < x < 20.41 and 5.7 < y < 6.1
        #20.1968 < x <  20.3466 and 5.758 < y < 6.1
        
        if (self.flag.data == False):
            self.velocity_publisher_tugbot_1.publish(self.commanded_velocities)
        self.flag_publisher_floor_1_completed.publish(self.flag)
        #print("\nX:")
        #print(self.coordinates.position.x)
        #print("\nY:")

def main(args=None):
    rclpy.init(args=args)
    node = Tugbot_1_Controller()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
