# spawn_robot_launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'my_robot', '-file', '/home/airlab/workspace/gazebo_scout2/urdf/scout_v2.urdf'],
             output='screen')
    ])