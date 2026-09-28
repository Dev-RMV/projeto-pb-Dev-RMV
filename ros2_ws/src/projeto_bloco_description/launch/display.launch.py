"""
URDF do Robo Sniper no RViz: robot_state_publisher + joint_state_publisher_gui + rviz2.

  ros2 launch projeto_bloco_description display.launch.py              # sliders das juntas
  ros2 launch projeto_bloco_description display.launch.py gui:=false   # juntas paradas em 0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('projeto_bloco_description')
    urdf = os.path.join(share, 'urdf', 'robo_sniper.urdf.xacro')
    gui = LaunchConfiguration('gui')
    descricao = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='joint_state_publisher_gui (sliders) em vez do comum'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': descricao}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'rviz', 'robo_sniper.rviz')]),
    ])
