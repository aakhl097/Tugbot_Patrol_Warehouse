#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import subprocess
import sys, select 
import string
import math

class Orientation_Tugbots(Node):
    def __init__(self):
        super().__init__("tugbots_bearings_extractor_node")
        qos = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, reliability = QoSReliabilityPolicy.RELIABLE, depth=10)

        self.orientation_publisher_tugbot_1 = self.create_publisher(Pose, 'orientation_tugbot_1', qos)
        self.orientation_publisher_tugbot_2 = self.create_publisher(Pose, 'orientation_tugbot_2', qos)
        self.orientation_publisher_tugbot_3 = self.create_publisher(Pose, 'orientation_tugbot_3', qos)

        command_tugbot_1_orientation = ['gz', 'topic', '--echo', '--topic', "/model/tugbot_1/pose"]
        command_tugbot_2_orientation = ['gz', 'topic', '--echo', '--topic', "/model/tugbot_2/pose"]
        command_tugbot_3_orientation = ['gz', 'topic', '--echo', '--topic', "/model/tugbot_3/pose"]

        self.orientation_tugbot_1 = Pose()
        self.orientation_tugbot_2 = Pose()
        self.orientation_tugbot_3 = Pose()

        self.message_tugbot_1 = ""
        self.bearing_tugbot_1 = ""
        self.capture_message_tugbot_1 = False
        self.capture_orientation_tugbot_1 = False
        self.quaternion_w_tugbot_1 = 0.0
        self.quaternion_z_tugbot_1 = 0.0
        self.quaternion_y_tugbot_1 = 0.0
        self.quaternion_x_tugbot_1 = 0.0

        self.message_tugbot_2 = ""
        self.bearing_tugbot_2 = ""
        self.capture_message_tugbot_2 = False
        self.capture_orientation_tugbot_2 = False
        self.quaternion_w_tugbot_2 = 0.0
        self.quaternion_z_tugbot_2 = 0.0
        self.quaternion_y_tugbot_2 = 0.0
        self.quaternion_x_tugbot_2 = 0.0

        self.message_tugbot_3 = ""
        self.bearing_tugbot_3 = ""
        self.capture_message_tugbot_3 = False
        self.capture_orientation_tugbot_3 = False
        self.quaternion_w_tugbot_3 = 0.0
        self.quaternion_z_tugbot_3 = 0.0
        self.quaternion_y_tugbot_3 = 0.0
        self.quaternion_x_tugbot_3 = 0.0
 
        self.process_tugbot_1 = subprocess.Popen(command_tugbot_1_orientation, stdout = subprocess.PIPE, stderr = subprocess.DEVNULL)
        self.process_tugbot_2 = subprocess.Popen(command_tugbot_2_orientation, stdout = subprocess.PIPE, stderr = subprocess.DEVNULL)
        self.process_tugbot_3 = subprocess.Popen(command_tugbot_3_orientation, stdout = subprocess.PIPE, stderr = subprocess.DEVNULL)

        timer = self.create_timer(1/30, self.timer_callback)

    def timer_callback(self):
        while select.select([self.process_tugbot_1.stdout], [], [], 0)[0]:
            line = self.process_tugbot_1.stdout.readline().decode().strip()
            if line.startswith('name: "tugbot_1"'):
                self.capture_message_tugbot_1 = True
                self.message_tugbot_1 = line + "\n"
            
            elif (self.capture_message_tugbot_1 == True):
                self.message_tugbot_1 += line + "\n"

                if line.startswith("orientation"):
                    self.capture_orientation_tugbot_1 = True

                if self.capture_orientation_tugbot_1 == True:
                    self.bearing_tugbot_1 += line + "\n"

                    if line.startswith("w:"):
                        self.capture_message_tugbot_1 = False
                        self.capture_orientation_tugbot_1 = False

                    filtered_line = self.bearing_tugbot_1.strip().split("\n")
                    
                    for line_splitted in filtered_line:
                        l = line_splitted.strip()
                        if l.startswith("x:"):
                            self.quaternion_x_tugbot_1 = float(l.split(":", 1)[1])
                        if l.startswith("y:"):
                            self.quaternion_y_tugbot_1 = float(l.split(":", 1)[1])
                        if l.startswith("z:"):
                            self.quaternion_z_tugbot_1 = float(l.split(":", 1)[1])
                        if l.startswith("w:"):
                            self.quaternion_w_tugbot_1 = float(l.split(":", 1)[1])

                        term_11 = 2*((self.quaternion_w_tugbot_1*self.quaternion_z_tugbot_1) +(self.quaternion_x_tugbot_1*self.quaternion_y_tugbot_1))
                        term_12 = 1 - ( 2 * (math.pow(self.quaternion_y_tugbot_1,2) + math.pow(self.quaternion_z_tugbot_1,2)) )
                        self.yaw = math.atan2(term_11, term_12)
                        self.orientation_tugbot_1.orientation.z = self.yaw
                        self.orientation_publisher_tugbot_1.publish(self.orientation_tugbot_1)
                        self.message_tugbot_1 = ""
                        self.bearing_tugbot_1 = ""

        while select.select([self.process_tugbot_2.stdout], [], [], 0)[0]:
            line_two = self.process_tugbot_2.stdout.readline().decode().strip()
            if line_two.startswith('name: "tugbot_2"'):
                self.capture_message_tugbot_2 = True
                self.message_tugbot_2 = line_two + "\n"
            
            elif (self.capture_message_tugbot_2 == True):
                self.message_tugbot_2 += line_two + "\n"

                if line_two.startswith("orientation"):
                    self.capture_orientation_tugbot_2 = True

                if self.capture_orientation_tugbot_2 == True:
                    self.bearing_tugbot_2 += line_two + "\n"

                    if line_two.startswith("w:"):
                        self.capture_message_tugbot_2 = False
                        self.capture_orientation_tugbot_2 = False

                    filtered_line_two = self.bearing_tugbot_2.strip().split("\n")
                    
                    for line_splitted_two in filtered_line_two:
                        l_two = line_splitted_two.strip()
                        if l_two.startswith("x:"):
                            self.quaternion_x_tugbot_2 = float(l_two.split(":", 1)[1])
                        if l_two.startswith("y:"):
                            self.quaternion_y_tugbot_2 = float(l_two.split(":", 1)[1])
                        if l_two.startswith("z:"):
                            self.quaternion_z_tugbot_2 = float(l_two.split(":", 1)[1])
                        if l_two.startswith("w:"):
                            self.quaternion_w_tugbot_2 = float(l_two.split(":", 1)[1])

                        term_21 = 2*((self.quaternion_w_tugbot_2*self.quaternion_z_tugbot_2) +(self.quaternion_x_tugbot_2*self.quaternion_y_tugbot_2))
                        term_22 = 1 - ( 2 * (math.pow(self.quaternion_y_tugbot_2,2) + math.pow(self.quaternion_z_tugbot_2,2)) )
                        self.yaw_tugbot_2 = math.atan2(term_21, term_22)
                        self.orientation_tugbot_2.orientation.z = self.yaw_tugbot_2
                        self.orientation_publisher_tugbot_2.publish(self.orientation_tugbot_2)
                        self.message_tugbot_2 = ""
                        self.bearing_tugbot_2 = ""


        while select.select([self.process_tugbot_3.stdout], [], [], 0)[0]:
            line_three = self.process_tugbot_3.stdout.readline().decode().strip()
            if line_three.startswith('name: "tugbot_3"'):
                self.capture_message_tugbot_3 = True
                self.message_tugbot_3 = line_three + "\n"
            
            elif (self.capture_message_tugbot_3 == True):
                self.message_tugbot_3 += line_three + "\n"

                if line_three.startswith("orientation"):
                    self.capture_orientation_tugbot_3 = True

                if self.capture_orientation_tugbot_3 == True:
                    self.bearing_tugbot_3 += line_three + "\n"

                    if line_three.startswith("w:"):
                        self.capture_message_tugbot_3 = False
                        self.capture_orientation_tugbot_3 = False

                    filtered_line_three = self.bearing_tugbot_3.strip().split("\n")
                    
                    for line_splitted_three in filtered_line_three:
                        l_three = line_splitted_three.strip()
                        if l_three.startswith("x:"):
                            self.quaternion_x_tugbot_3 = float(l_three.split(":", 1)[1])
                        if l_three.startswith("y:"):
                            self.quaternion_y_tugbot_3 = float(l_three.split(":", 1)[1])
                        if l_three.startswith("z:"):
                            self.quaternion_z_tugbot_3 = float(l_three.split(":", 1)[1])
                        if l_three.startswith("w:"):
                            self.quaternion_w_tugbot_3 = float(l_three.split(":", 1)[1])

                        term_31 = 2*((self.quaternion_w_tugbot_3*self.quaternion_z_tugbot_3) +(self.quaternion_x_tugbot_3*self.quaternion_y_tugbot_3))
                        term_32 = 1 - ( 2 * (math.pow(self.quaternion_y_tugbot_3,2) + math.pow(self.quaternion_z_tugbot_3,2)) )
                        self.yaw_tugbot_3 = math.atan2(term_31, term_32)
                        self.orientation_tugbot_3.orientation.z = self.yaw_tugbot_3
                        self.orientation_publisher_tugbot_3.publish(self.orientation_tugbot_3)
                        self.message_tugbot_3 = ""
                        self.bearing_tugbot_3 = ""

def main (args=None):
    rclpy.init(args=args)
    node = Orientation_Tugbots()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()