"""Launch de exemplo: dois nós, parâmetro sobrescrito — base para o seu bringup.launch.py."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='pacote_minimo', executable='no_publisher',
             parameters=[{'frequencia_hz': 2.0}]),
        Node(package='pacote_minimo', executable='no_subscriber'),
    ])
