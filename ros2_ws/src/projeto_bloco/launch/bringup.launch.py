"""
TP3: bringup unico do Robo Sniper. Um comando sobe, EM SEQUENCIA:

  1. Gazebo Classic 11 com o mundo do projeto (sala com obstaculos e alvos impressos com a arte
     real) e o robot_state_publisher com o URDF (xacro);
  2. spawn do robo no Gazebo (camera -> /camera/image_raw, lidar -> /scan, esteiras /cmd_vel ->
     /odom + TF odom->base_footprint, rodas -> /gazebo/joint_states);
  3. assim que o spawn termina: joint_state_publisher (junta rodas do Gazebo + torre do
     servidor_engajar em /joint_states), SLAM Toolbox (/scan -> /map, TF map->odom) e os nos de
     percepcao do TP2 (rastreador_yolo + servidor_engajar) mais o segmentador_yolo do TP3
     (/vision/segmented), todos lendo a camera simulada;
  4. RViz2 com a configuracao salva (rviz/bringup.rviz: mapa, scan, TF, robo e as imagens);
  5. 3 s depois, o piloto_demo sobe; apos a espera inicial (30 s, config/params.yaml) ele percorre a
     rota (quadrado de 1,4 m e um giro completo) para o mapa ser construido;
  6. 10 s depois do spawn, o cliente_engajar arma a torre e envia os goals de engajamento
     (config/bringup.yaml: tres goals na classe "alvo"), com o robo ainda parado olhando para os
     alvos: a torre mira e "dispara" (/torre/visao mostra MIRANDO -> TRAVADO -> DISPARO).
     engajar:=false deixa isso para `ros2 run projeto_bloco cliente_engajar` a mao.

Parametros: config/params.yaml (nos) + config/bringup.yaml (use_sim_time para todos e ajustes da
simulacao) + config/slam.yaml (SLAM Toolbox). Rastreador e segmentador precisam do venv do YOLO
(ros2_ws/.venv, criado pelo scripts/setup.sh): o launch o poe no PYTHONPATH sozinho, achando-o a
partir da pasta de instalacao do pacote (<ws>/install/projeto_bloco -> <ws>/.venv); sem ele os dois
nos saem logo no inicio com "ultralytics nao encontrado" e as imagens /vision/* nao aparecem.

  ros2 launch projeto_bloco bringup.launch.py                       # tudo
  ros2 launch projeto_bloco bringup.launch.py gui:=false            # sem a janela do Gazebo
  ros2 launch projeto_bloco bringup.launch.py piloto:=false         # dirigir com o teleop
  ros2 launch projeto_bloco bringup.launch.py slam:=false rviz:=false
Outro terminal: ros2 run tf2_tools view_frames | ros2 run projeto_bloco cliente_engajar |
  ros2 run nav2_map_server map_saver_cli -f mapa --ros-args -p use_sim_time:=true
"""

import glob
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def pythonpath_com_venv():
    """PYTHONPATH com o site-packages de <ws>/.venv (venv do YOLO) na frente, se ele existir.

    <ws> e achado subindo a partir da instalacao do pacote (<ws>/install/projeto_bloco), como o
    ros2_ws/ambiente.sh faz; sem o venv devolve o PYTHONPATH atual (e os nos do YOLO avisam).
    """
    atual = os.environ.get('PYTHONPATH', '')
    pasta = get_package_prefix('projeto_bloco')
    for _ in range(3):
        pasta = os.path.dirname(pasta)
        for site in sorted(glob.glob(os.path.join(pasta, '.venv', 'lib', 'python3*', 'site-packages'))):
            if site not in atual.split(os.pathsep):
                return site + (os.pathsep + atual if atual else '')
            return atual
    return atual


def generate_launch_description():
    share = get_package_share_directory('projeto_bloco')
    desc = get_package_share_directory('projeto_bloco_description')
    params = os.path.join(share, 'config', 'params.yaml')
    bringup = os.path.join(share, 'config', 'bringup.yaml')
    urdf = os.path.join(desc, 'urdf', 'robo_sniper.urdf.xacro')
    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch',
                                 'gazebo.launch.py')
    sim_time = {'use_sim_time': True}
    show = LaunchConfiguration('show')

    argumentos = [
        DeclareLaunchArgument('world', default_value=os.path.join(desc, 'worlds', 'sala_alvos.world'),
                              description='mundo do Gazebo'),
        DeclareLaunchArgument('gui', default_value='true', description='janela do Gazebo (gzclient)'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz2'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'rviz', 'bringup.rviz')),
        DeclareLaunchArgument('slam', default_value='true', description='SLAM Toolbox'),
        DeclareLaunchArgument('slam_params', default_value=os.path.join(share, 'config', 'slam.yaml')),
        DeclareLaunchArgument('piloto', default_value='true',
                              description='piloto_demo percorre a rota (false = teleop a mao)'),
        DeclareLaunchArgument('engajar', default_value='true',
                              description='cliente_engajar arma a torre e envia os goals do '
                                          'config/bringup.yaml 10 s apos o spawn (false = so a mao)'),
        DeclareLaunchArgument('show', default_value='false', description='janelas cv2.imshow'),
    ]

    ambiente = [
        # venv do YOLO (ultralytics + torch) no PYTHONPATH dos nos, mesmo sem `source ambiente.sh`
        SetEnvironmentVariable('PYTHONPATH', pythonpath_com_venv()),
        # Fast-DDS com segmento SHM maior: sem isso frames grandes chegam a ~5 fps (ver o XML)
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                               os.path.join(share, 'config', 'fastdds_profile.xml')),
        # model://alvo_impresso (tambem exportado no package.xml do projeto_bloco_description)
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', [os.path.join(desc, 'models'), ':',
                                                     EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                         default_value='')]),
    ]

    # ---- 1. Gazebo + robot_state_publisher ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gui')}.items())
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf]),
                                                                value_type=str), **sim_time}])

    # ---- 2. spawn (espera o gzserver sozinho) ----
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-topic', 'robot_description', '-entity', 'robo_sniper', '-z', '0.02'])

    # ---- 3. depois do spawn: juntas, SLAM e percepcao ----
    jsp = Node(package='joint_state_publisher', executable='joint_state_publisher',
               parameters=[{'source_list': ['/gazebo/joint_states', '/torre/joint_states'],
                            'rate': 20, **sim_time}])
    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                output='screen', parameters=[LaunchConfiguration('slam_params'), sim_time],
                condition=IfCondition(LaunchConfiguration('slam')))
    rastreador = Node(package='projeto_bloco', executable='rastreador_yolo', output='screen',
                      parameters=[params, bringup, {'show': show}])
    segmentador = Node(package='projeto_bloco', executable='segmentador_yolo', output='screen',
                       parameters=[params, bringup, {'show': show}])
    servidor = Node(package='projeto_bloco', executable='servidor_engajar', output='screen',
                    parameters=[params, bringup, {'show': show}])

    # ---- 4. RViz ----
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[sim_time],
                condition=IfCondition(LaunchConfiguration('rviz')))

    # ---- 5. piloto da demo (rota do params.yaml), 3 s depois do SLAM ----
    piloto = TimerAction(period=3.0, actions=[
        Node(package='projeto_bloco', executable='piloto_demo', output='screen',
             parameters=[params, bringup], condition=IfCondition(LaunchConfiguration('piloto')))])

    # ---- 6. engajamento automatico: o action client arma a torre e envia os goals (classes do
    #         bringup.yaml) 10 s depois do spawn, com o robo ainda parado olhando para os alvos ----
    engajador = TimerAction(period=10.0, actions=[
        Node(package='projeto_bloco', executable='cliente_engajar', output='screen',
             parameters=[params, bringup], condition=IfCondition(LaunchConfiguration('engajar')))])

    depois_do_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[LogInfo(msg='[bringup] robo no Gazebo: subindo juntas, SLAM, percepcao, RViz, '
                             'piloto e engajamento'),
                 jsp, slam, rastreador, segmentador, servidor, rviz, piloto, engajador]))

    return LaunchDescription(argumentos + ambiente + [
        LogInfo(msg='[bringup] 1/5 Gazebo + robot_state_publisher'),
        gazebo, rsp,
        LogInfo(msg='[bringup] 2/5 spawn do robo (aguarda o gzserver)'),
        spawn,
        depois_do_spawn,
    ])
