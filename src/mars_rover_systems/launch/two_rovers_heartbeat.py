from launch import LaunchDescription
from launch_ros.actions import Node

"""
return two nodes using package_name, executable file for each node
type of output channel for printing the program's output
TO display info message by color set emulate_tty= True
"""

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'mars_rover_systems',      
            executable = 'heartbeat_executable',
            output = 'screen',
            emulate_tty = True
        ),

        Node(
            package = 'mars_rover_systems',
            executable = 'heartbeat_executable_2',
            output = 'screen',
            emulate_tty = True
        ),
        Node(
            package ='mars_rover_systems',
            executable = 'temperature_executable',
            output = 'screen',
            emulate_tty = True
        )
    ])