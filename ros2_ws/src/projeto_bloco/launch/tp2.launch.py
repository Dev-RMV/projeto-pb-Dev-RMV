"""
TP2: camera -> rastreador YOLO (+Kalman) -> action server de engajamento, com o URDF no RViz.

  camera_publisher --/camera/image_raw--> rastreador_yolo --/vision/deteccoes--> servidor_engajar
  servidor_engajar --/torre/joint_states--> joint_state_publisher -> robot_state_publisher (TF)
  action /engajar_alvo e servico /arma/controle: use o cliente em outro terminal:
    ros2 run projeto_bloco cliente_engajar

  ros2 launch projeto_bloco tp2.launch.py show:=true rviz:=true     # video de alvos (padrao)
  ros2 launch projeto_bloco tp2.launch.py source:=0                  # camera 0
Sem source:=, usa midia/alvos_dv20.mp4 do workspace (achado subindo a partir da instalacao do
pacote: <ws>/install/projeto_bloco -> <ws>/midia).
O rastreador precisa do venv do YOLO no PYTHONPATH: `source ros2_ws/ambiente.sh` antes.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

VIDEO_PADRAO = os.path.join('midia', 'alvos_dv20.mp4')


def video_padrao():
    """Caminho de <ws>/midia/alvos_dv20.mp4, ou '' se nao achar (a camera avisa e sai)."""
    pasta = get_package_prefix('projeto_bloco')   # <ws>/install/projeto_bloco (ou <ws>/install)
    for _ in range(3):
        pasta = os.path.dirname(pasta)
        video = os.path.join(pasta, VIDEO_PADRAO)
        if os.path.isfile(video):
            return video
    return ''


def generate_launch_description():
    share = get_package_share_directory('projeto_bloco')
    params = os.path.join(share, 'config', 'params.yaml')
    urdf = os.path.join(get_package_share_directory('projeto_bloco_description'),
                        'urdf', 'robo_sniper.urdf.xacro')
    show = LaunchConfiguration('show')

    return LaunchDescription([
        # Fast-DDS com segmento SHM maior: sem isso frames grandes chegam a ~5 fps (ver o XML)
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                               os.path.join(share, 'config', 'fastdds_profile.xml')),
        DeclareLaunchArgument('source', default_value=video_padrao(),
                              description="'0','1',.. = camera; caminho = arquivo de video"),
        DeclareLaunchArgument('show', default_value='false', description='janelas cv2.imshow'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz com URDF e imagens'),

        Node(package='projeto_bloco', executable='camera_publisher', output='screen',
             parameters=[params, {'source': LaunchConfiguration('source')}]),
        Node(package='projeto_bloco', executable='rastreador_yolo', output='screen',
             parameters=[params, {'show': show}]),
        Node(package='projeto_bloco', executable='servidor_engajar', output='screen',
             parameters=[params, {'show': show}]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf]),
                                                              value_type=str)}]),
        # junta os estados da torre (servidor_engajar) com as rodas (paradas em 0) em /joint_states
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'source_list': ['/torre/joint_states'], 'rate': 20}]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'rviz', 'tp2.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
