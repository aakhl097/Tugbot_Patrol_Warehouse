#!/usr/bin/env python3 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():

    use_sime_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'true',
            description = 'Use Gazebo Clock if true'),

        Node(
            package = 'driver_nodes',
            executable = 'Tugbots_Positions_Extractor_Node',
            name = 'Tugbots_Positions_Extractor_Node',
            output = 'screen'),

        Node(
            package = 'driver_nodes',
            executable = 'Tugbots_Bearings_Extractor_Node',
            name = 'Tugbots_Bearings_Extractor_Node',
            output = 'screen'),
            
        Node(
            package = 'driver_nodes',
            executable = 'Tugbot_1_Controller_Node',
            name = 'Tugbot_1_Controller_Node',
            output = 'screen'),

        Node(
            package = 'driver_nodes',
            executable = 'Tugbot_1_Second_Floor_Patrol',
            name = 'Tugbot_1_Second_Floor_Patrol',
            output = 'screen'),

        Node(
            package = 'driver_nodes',
            executable = 'Tugbot_2_Controller_Node',
            name = 'Tugbot_2_Controller_Node',
            output = 'screen'),

        Node(
            package = 'driver_nodes',
            executable = 'Tugbot_3_Controller_Node',
            name = 'Tugbot_3_Controller_Node',
            output = 'screen'),

        Node(
            package = 'driver_nodes',
            executable = 'LiDAR_Readings_Extractor_Node',
            name = 'LiDAR_Readings_Extractor_Node',
            output = 'screen'),
    ])
