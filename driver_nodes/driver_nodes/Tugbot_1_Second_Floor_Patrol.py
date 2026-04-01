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
        super().__init__("tugbot_1_controller_second_floor")
        qos = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, reliability = QoSReliabilityPolicy.RELIABLE, depth=10)
        self.lidar_readings = LaserScan()
        self.coordinates = Pose()
        self.bearings = Pose()
        self.commanded_velocities = Twist()
        self.signal = Bool()
        self.velocity_publisher_tugbot_1 = self.create_publisher(Twist, 'model/tugbot_1/cmd_vel', qos)
        self.flag_listener = self.create_subscription(Bool, 'flag_floor_1_completed', self.flag_callback, qos)
        self.position_listener_tugbot_1 = self.create_subscription(Pose, 'cartesian_position_tugbot_1', self.position_callback, qos)
        self.orientation_listener_tugbot_1 = self.create_subscription(Pose, 'orientation_tugbot_1', self.orientation_callback, qos)
        self.lidar_measurements = self.create_subscription(LaserScan, 'front_lidar_scan_tugbot_1', self.lidar_callback_tugbot_1, qos)
        self.collision_alert_right = False
        self.collision_alert_left = False
        self.deactivate_lidar = True
        
        self.exit_elevator = False
        self.move_to_left_floor = False
        self.move_to_right_floor = False

        self.rotate_left_q1_q2 = False
        self.rotate_left_q2_q3 = False
        self.rotate_left_q3_q4 = False
        self.rotate_left_q4_q1 = False  
        self.drive_straight_left_front_floor = False

        self.rotate_right_q1_q2 = False
        self.rotate_right_q1_q4 = False
        self.rotate_right_q4_q3 = False
        self.rotate_right_q3_q2 = False
        self.drive_straight_right_front_floor = False

        self.second_collision_alert = False
        self.speed_adjustment = False
        self.return_to_elevator = False

        self.timer = self.create_timer(1/30, self.timer_callback)
        
        #x = 7.19, y =1.0998

    def flag_callback(self, msg):
        self.signal = msg
    
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

                if (all_distances[320] >= 0.01 and all_distances[320] <= 3.0):
                    self.collision_alert_right = True
                #elif (all_distances[370] >= 0.1 and all_distances[370] <= 2.5):
                    #self.collision_alert = True
                #elif (all_distances[350] >= 0.1 and all_distances[350] <= 2.5):
                    #self.collision_alert = True
                elif (all_distances[400] >= 0.01 and all_distances[400] <= 3.0):
                    self.collision_alert_left = True
                else:
                    self.collision_alert_right = False
                    self.collision_alert_left = False

    def timer_callback(self):

        if self.signal.data == True: #Reached Second Floor Flag

            if self.deactivate_lidar == False:
                thread_lidar = threading.Thread(target = self.collision_detection)
                thread_lidar.start()

            if (self.coordinates.position.z >= 4.9):

                if (self.exit_elevator == False):
                    self.commanded_velocities.angular.z = 0.0
                    self.commanded_velocities.linear.x = 0.33
                    #exit when 21.3 <= x <= 21.5 and 5.5 <= y <= 6.0

                    if ((self.coordinates.position.x >=21.3) and (self.coordinates.position.x <=21.5) and (self.coordinates.position.y >=5.5) and (self.coordinates.position.y <=6.0)):
                        self.commanded_velocities.linear.x = 0.0
                        self.exit_elevator = True
                        self.rotate_left_q1_q2 = True
                        self.deactivate_lidar = False

                if (self.rotate_left_q1_q2 == True):

                    flag = False
                    if (self.coordinates.position.y >=5.5) and (self.coordinates.position.y <=6.0):
                        self.commanded_velocities.angular.z = 0.2
                        self.commanded_velocities.linear.x = 0.0
                        flag = True
                    elif (self.coordinates.position.y >= -5.99 and self.coordinates.position.y <= -5.3):
                        self.commanded_velocities.angular.z = 0.2
                        self.commanded_velocities.linear.x = 0.0
                        flag = False
                    else:
                        self.commanded_velocities.angular.z = 0.0
                        flag = False

                    if ((self.bearings.orientation.z > 1.58) and (self.bearings.orientation.z <=3.11) ):
                        self.commanded_velocities.angular.z = -0.5
                    
                    if ((self.bearings.orientation.z <= 1.57) and (self.bearings.orientation.z >= 1.5)):
                        self.commanded_velocities.angular.z = 0.0
                        if ((self.collision_alert_right == True) and (self.return_to_elevator==True)):
                            self.commanded_velocities.angular.z = 0.5

                        if self.speed_adjustment == False:
                            self.commanded_velocities.linear.x = 0.6
                        else:
                            self.commanded_velocities.linear.x = 1.0

                        if ((self.return_to_elevator == True) and (flag == True)):
                            self.commanded_velocities.linear.x = 0.0
                            self.commanded_velocities.angular.z = 0.0

                        if ((self.coordinates.position.y <=7.96
                        ) and (self.coordinates.position.y >=7.5)):
                            self.commanded_velocities.linear.x = 0.0
                            self.rotate_left_q1_q2 = False
                            self.rotate_left_q2_q3 = True

                if (self.rotate_left_q2_q3 == True):

                    self.commanded_velocities.angular.z = 0.15
                    self.commanded_velocities.linear.x = 0.0

                    if ((self.bearings.orientation.z <= 3.11) and (self.bearings.orientation.z >= 2.99)):
                        self.commanded_velocities.angular.z = 0.0
                        self.commanded_velocities.linear.x = 0.0
                        self.rotate_left_q2_q3 = False
                        self.rotate_left_q3_q4 = True
                        self.deactivate_lidar = False

                if (self.rotate_left_q3_q4 == True):
                    self.commanded_velocities.angular.z = 0.0
                    self.commanded_velocities.linear.x = 1.4

                    if self.collision_alert_right == True:
                        self.commanded_velocities.angular.z = 0.5
                        #self.collision_alert_right = False
                    
                    #if self.collision_alert_left == True:
                        #self.commanded_velocities.angular.z = -0.5
                        #self.collision_alert_left = False

                    if ((self.bearings.orientation.z >= -3.11) and (self.bearings.orientation.z <=-1.0) ):
                        self.commanded_velocities.angular.z = -0.5

                    #if ((self.bearings.orientation.z <= 2.9) and (self.bearings.orientation.z >= 2.0)):
                    #    self.commanded_velocities.angular.z = 0.5

                    if ((self.coordinates.position.x >= -7.3) and (self.coordinates.position.x <=-6.2)):
                        self.commanded_velocities.angular.z = 0.2
                        self.commanded_velocities.linear.x = 0.0
                        self.rotate_left_q3_q4 = False
                        self.rotate_left_q4_q1 = True

                if (self.rotate_left_q4_q1 == True):
                    if ((self.bearings.orientation.z >= -0.02) and (self.bearings.orientation.z <= 0.15)):
                        self.commanded_velocities.angular.z = 0.0
                        self.commanded_velocities.linear.x = 1.6
                        self.rotate_left_q4_q1 = False
                        self.drive_straight_left_front_floor = True

                
                if self.drive_straight_left_front_floor == True:
                    self.commanded_velocities.angular.z = 0.0
                    self.commanded_velocities.linear.x = 1.6

                    if self.collision_alert_left == True:
                        self.commanded_velocities.angular.z = -0.5

                    if ((self.bearings.orientation.z <= -0.01) and (self.bearings.orientation.z >=-1.5) ):
                        self.commanded_velocities.angular.z = 0.5

                    if ((self.coordinates.position.x >=21.15) and (self.coordinates.position.x <=21.33)): #21.28 and 21.7
                        self.drive_straight_left_front_floor = False
                        self.rotate_right_q1_q2 = True
                
                if (self.rotate_right_q1_q2 == True):
                    self.commanded_velocities.angular.z = -0.2
                    self.commanded_velocities.linear.x = 0.0
                    if ((self.bearings.orientation.z >= -2.0) and (self.bearings.orientation.z <= -1.5)): #threshold was -1.7 before.
                        self.commanded_velocities.angular.z = 0.0
                        self.commanded_velocities.linear.x = 2.0
                        self.rotate_right_q1_q2 = False
                        self.move_to_right_floor = True
                
                if (self.move_to_right_floor == True):
                    self.commanded_velocities.linear.x = 1.6

                    if self.collision_alert_left == True:
                        self.commanded_velocities.angular.z = -0.5

                    if ((self.bearings.orientation.z <= -1.55) and (self.bearings.orientation.z >=-2.8) ):
                        self.commanded_velocities.angular.z = 0.5

                    if (self.coordinates.position.y >= -5.7 and self.coordinates.position.y <= -5.3):
                        self.commanded_velocities.linear.x = 0.0
                        self.move_to_right_floor = False
                        self.rotate_right_q1_q4 = True

                if (self.rotate_right_q1_q4 == True):
                    self.commanded_velocities.angular.z = -0.2
                    #if ((self.bearings.orientation.z <= 3.11) and (self.bearings.orientation.z >= 2.99)):
                    if ((self.bearings.orientation.z <= -2.99) and (self.bearings.orientation.z >= -3.12)):
                        self.commanded_velocities.angular.z = -0.0
                        self.commanded_velocities.linear.x = 1.6
                        self.rotate_right_q1_q4 = False
                        self.rotate_right_q4_q3 = True

                if (self.rotate_right_q4_q3 == True):
                    self.commanded_velocities.angular.z = 0.0
                    self.commanded_velocities.linear.x = 2.0

                    if self.collision_alert_left == True:
                        self.commanded_velocities.angular.z = -0.5

                    if ((self.bearings.orientation.z <= 3.12) and (self.bearings.orientation.z >=2.91) ):
                        self.commanded_velocities.angular.z = 0.5
                        
                    if ((self.coordinates.position.x >= -6.7) and (self.coordinates.position.x <=-6.2)):
                        self.rotate_right_q4_q3 = False
                        self.rotate_right_q3_q2 = True

                if self.rotate_right_q3_q2 == True:
                    self.commanded_velocities.angular.z = 0.2
                    self.commanded_velocities.linear.x = 0.0
                    if ((self.bearings.orientation.z >= -0.15) and (self.bearings.orientation.z <= -0.01)):
                        self.commanded_velocities.angular.z = 0.0
                        self.rotate_right_q3_q2 = False
                        self.drive_straight_right_front_floor = True

                if self.drive_straight_right_front_floor == True:
                    self.commanded_velocities.angular.z = 0.0
                    self.commanded_velocities.linear.x = 2.1

                    if self.collision_alert_right == True:
                        self.commanded_velocities.angular.z = 0.5
        
                    if ((self.bearings.orientation.z >= 0.07) and (self.bearings.orientation.z <= 1.5) ):
                        self.commanded_velocities.angular.z = -0.5

                    if ((self.coordinates.position.x >=21.05) and (self.coordinates.position.x <=21.23)):
                        self.return_to_elevator = True
                        self.speed_adjustment = True
                        self.rotate_left_q1_q2 = True
                        self.drive_straight_right_front_floor = False
                    
                # x= 21.5796 and  y = -5.7072
                #self.exit_elevator = True
            
            self.velocity_publisher_tugbot_1.publish(self.commanded_velocities)
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
