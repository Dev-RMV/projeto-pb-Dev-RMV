"""
Sobe os 3 nos: camera_publisher -> color_segmenter e face_features.

  ros2 launch projeto_bloco projeto_bloco.launch.py source:=/caminho/demo.mp4 show:=true
  ros2 launch projeto_bloco projeto_bloco.launch.py source:=0      # camera 0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('projeto_bloco')
    params = os.path.join(share, 'config', 'params.yaml')
    source = LaunchConfiguration('source')
    show = LaunchConfiguration('show')

    return LaunchDescription([
        # Fast-DDS com segmento SHM maior: sem isso frames grandes chegam a ~5 fps (ver o XML)
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                               os.path.join(share, 'config', 'fastdds_profile.xml')),
        DeclareLaunchArgument('source', default_value='',
                              description="'0','1',.. = camera; caminho = arquivo de video"),
        DeclareLaunchArgument('show', default_value='false', description='janelas cv2.imshow'),

        Node(package='projeto_bloco', executable='camera_publisher', output='screen',
             parameters=[params, {'source': source}]),
        Node(package='projeto_bloco', executable='color_segmenter', output='screen',
             parameters=[params, {'show': show}]),
        Node(package='projeto_bloco', executable='face_features', output='screen',
             parameters=[params, {'show': show}]),
    ])
