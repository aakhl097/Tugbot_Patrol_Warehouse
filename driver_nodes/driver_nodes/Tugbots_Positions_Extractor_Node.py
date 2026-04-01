#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import subprocess
import sys, select 
import string

class Position_Tugbots(Node):
    def __init__(self):
        super().__init__("tugbots_positions_extractor_node")
        qos = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, reliability = QoSReliabilityPolicy.RELIABLE, depth=10)
        
        # gz sim tugbot_elevator.sdf 
        # ros2 run ros_gz_bridge parameter_bridge /model/tugbot_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
        # ros2 topic pub --once /model/tugbot_2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}}"
        # Run gz topic -l command to see list of active gazebo topics. 
        #Note that the command below moves the platform.
        # gz topic -t "/model/prismatic_box_mechanism/joint/box_slider_joint/cmd_vel" -m gz.msgs.Double -p "data: 0.2"

        self.position_publisher_tugbot_1 = self.create_publisher(Pose, 'cartesian_position_tugbot_1', qos)
        self.position_publisher_tugbot_2 = self.create_publisher(Pose, 'cartesian_position_tugbot_2', qos)
        self.position_publisher_tugbot_3 = self.create_publisher(Pose, 'cartesian_position_tugbot_3', qos)

        command_tugbot_1_position = ['gz', 'topic', '--echo', '--topic', "/model/tugbot_1/pose"]
        command_tugbot_2_position = ['gz', 'topic', '--echo', '--topic', "/model/tugbot_2/pose"]
        command_tugbot_3_position = ['gz', 'topic', '--echo', '--topic', "/model/tugbot_3/pose"]

        self.position_tugbot_1 = Pose()
        self.position_tugbot_2 = Pose()
        self.position_tugbot_3 = Pose()

        self.capture_position_tugbot_1 = False
        self.capture_position_tugbot_2 = False
        self.capture_position_tugbot_3 = False
        
        self.process_tugbot_1 = subprocess.Popen(command_tugbot_1_position, stdout = subprocess.PIPE, stderr = subprocess.DEVNULL)
        self.process_tugbot_2 = subprocess.Popen(command_tugbot_2_position, stdout = subprocess.PIPE, stderr = subprocess.DEVNULL)
        self.process_tugbot_3 = subprocess.Popen(command_tugbot_3_position, stdout = subprocess.PIPE, stderr = subprocess.DEVNULL)

        timer = self.create_timer(1/30, self.timer_callback)

    def timer_callback(self):
        while select.select([self.process_tugbot_1.stdout], [], [], 0)[0]:
            line = self.process_tugbot_1.stdout.readline().decode().strip()
            if line.startswith('name: "tugbot_1"'):
                self.capture_position_tugbot_1 = True
                self.message = line + "\n"
            
            elif (self.capture_position_tugbot_1 == True):
                self.message += line + "\n"

                if line.startswith("orientation"):
                    self.capture_position_tugbot_1 = False
                    filtered_line = self.message.strip().split("\n")
                    for line_splitted in filtered_line:
                        l = line_splitted.strip()
                        if l.startswith("x:"):
                            self.position_tugbot_1.position.x = float(l.split(":", 1)[1])
                            #print(l)
                            #print(self.position_tugbot_1.position.x)
                        if l.startswith("y:"):
                            self.position_tugbot_1.position.y = float(l.split(":", 1)[1])
                        
                        if l.startswith("z:"):
                            self.position_tugbot_1.position.z = float(l.split(":", 1)[1])

                        self.position_publisher_tugbot_1.publish(self.position_tugbot_1)


        while select.select([self.process_tugbot_2.stdout], [], [], 0)[0]:
            line_two = self.process_tugbot_2.stdout.readline().decode().strip()
            if line_two.startswith('name: "tugbot_2"'):
                self.capture_position_tugbot_2 = True
                self.message_two = line_two + "\n"
            
            elif (self.capture_position_tugbot_2 == True):
                self.message_two += line_two + "\n"

                if line_two.startswith("orientation"):
                    self.capture_position_tugbot_2 = False
                    filtered_line_two = self.message_two.strip().split("\n")
                    for line_splitted_two in filtered_line_two:
                        l_two = line_splitted_two.strip()
                        if l_two.startswith("x:"):
                            self.position_tugbot_2.position.x = float(l_two.split(":", 1)[1])
                            #print(l)
                            #print(self.position_tugbot_1.position.x)
                        if l_two.startswith("y:"):
                            self.position_tugbot_2.position.y = float(l_two.split(":", 1)[1])
                        
                        self.position_publisher_tugbot_2.publish(self.position_tugbot_2)

        while select.select([self.process_tugbot_3.stdout], [], [], 0)[0]:
            line_three = self.process_tugbot_3.stdout.readline().decode().strip()
            if line_three.startswith('name: "tugbot_3"'):
                self.capture_position_tugbot_3 = True
                self.message_three = line_three + "\n"
            
            elif (self.capture_position_tugbot_3 == True):
                self.message_three += line_three + "\n"

                if line_three.startswith("orientation"):
                    self.capture_position_tugbot_3 = False
                    filtered_line_three = self.message_three.strip().split("\n")
                    for line_splitted_three in filtered_line_three:
                        l_three = line_splitted_three.strip()
                        if l_three.startswith("x:"):
                            self.position_tugbot_3.position.x = float(l_three.split(":", 1)[1])
                            #print(l)
                            #print(self.position_tugbot_1.position.x)
                        if l_three.startswith("y:"):
                            self.position_tugbot_3.position.y = float(l_three.split(":", 1)[1])
                        
                        self.position_publisher_tugbot_3.publish(self.position_tugbot_3)

def main (args=None):
    rclpy.init(args=args)
    node = Position_Tugbots()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()