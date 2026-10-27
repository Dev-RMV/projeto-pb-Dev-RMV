"""
Robo Sniper no Gazebo Classic 11: mundo com alvos, robot_state_publisher e spawn do URDF.
Sensores simulados: /camera/image_raw, /scan; esteiras: /cmd_vel -> /odom (+ TF odom->base_footprint).
As rodas chegam em /gazebo/joint_states e o joint_state_publisher as junta (com /torre/joint_states,
se o servidor_engajar estiver no ar) em /joint_states para o robot_state_publisher.

  ros2 launch projeto_bloco_description gazebo.launch.py                 # Gazebo + RViz
  ros2 launch projeto_bloco_description gazebo.launch.py gui:=false rviz:=false   # so o servidor
  ros2 launch projeto_bloco_description gazebo.launch.py world:=<share>/worlds/sala_alvos.world
  ros2 run teleop_twist_keyboard teleop_twist_keyboard                  # dirigir (se instalado)
O TP3 (SLAM + percepcao) usa este mesmo Gazebo pelo projeto_bloco/bringup.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('projeto_bloco_description')
    urdf = os.path.join(share, 'urdf', 'robo_sniper.urdf.xacro')
    descricao = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    gazebo = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        # modelos proprios (model://alvo_impresso): tambem exportado no package.xml (gazebo_ros)
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', [os.path.join(share, 'models'), ':',
                                                     EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                         default_value='')]),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'alvos.world')),
        DeclareLaunchArgument('gui', default_value='true', description='janela do Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'gui': LaunchConfiguration('gui')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': descricao, 'use_sim_time': True}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'source_list': ['/gazebo/joint_states', '/torre/joint_states'],
                          'rate': 20, 'use_sim_time': True}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-topic', 'robot_description', '-entity', 'robo_sniper', '-z', '0.02']),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'rviz', 'robo_sniper_gazebo.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
