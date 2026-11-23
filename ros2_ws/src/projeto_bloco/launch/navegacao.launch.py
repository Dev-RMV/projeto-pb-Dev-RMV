"""
TP4: navegacao autonoma do Robo Sniper com o Nav2, tudo com um comando.

O launch sobe, EM SEQUENCIA:

  1. Gazebo Classic 11 com o mundo sala_patrulha.world (a sala do mapa, mais um caixote que nao
     esta no mapa) e o robot_state_publisher com o URDF;
  2. spawn do robo na origem (lidar -> /scan, esteiras /cmd_vel -> /odom, TF odom->base_footprint);
  3. assim que o spawn termina: joint_state_publisher, o Nav2 (nav2_bringup/bringup_launch.py com
     config/nav2.yaml: map_server com o mapa salvo, AMCL, costmaps global e local, planner,
     controller, behavior server e bt_navigator), a percepcao do TP2 (rastreador_yolo +
     servidor_engajar), o RViz2 (rviz/navegacao.rviz), a behavior tree (arvore_navegacao), o
     obstaculo_surpresa e a patrulha, que espera o Nav2 ficar ativo e percorre os postos de tiro;
  4. com bag:=<pasta>, um `ros2 bag record` grava a sessao inteira nessa pasta;
  5. com encerrar:=true, o launch inteiro termina quando a patrulha acaba (fecha o bag).

Parametros: config/params.yaml (nos do projeto) + config/navegacao.yaml (use_sim_time para todos
e ajustes da simulacao) + config/nav2.yaml (Nav2).

  ros2 launch projeto_bloco navegacao.launch.py                      # patrulha com a behavior tree
  ros2 launch projeto_bloco navegacao.launch.py arvore:=false surpresa:=false  # so Nav2 + patrulha
  ros2 launch projeto_bloco navegacao.launch.py patrulhar:=false     # goals a mao (RViz: 2D Goal)
  ros2 launch projeto_bloco navegacao.launch.py bag:=$HOME/bag_patrulha csv:=$HOME/patrulha.csv \
      encerrar:=true
Outro terminal (source ros2_ws/ambiente.sh):
  ros2 action send_goal /navegar_com_pausa nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.3}, orientation: {w: 1.0}}}}"
"""

import glob
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, EnvironmentVariable, LaunchConfiguration,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# topicos gravados no bag: sensores, odometria, TF, comando das esteiras, localizacao, plano,
# costmaps nao (grandes) e os topicos de estado da arvore, da patrulha e do obstaculo surpresa
TOPICOS_BAG = ['/scan', '/odom', '/tf', '/tf_static', '/cmd_vel', '/amcl_pose', '/plan', '/map',
               '/arvore/estado', '/arvore/distancia_frontal', '/patrulha/status',
               '/patrulha/rota', '/obstaculo_surpresa/status', '/arma/disparo']


def pythonpath_com_venv():
    """PYTHONPATH com o site-packages de <ws>/.venv (venv do YOLO) na frente, se ele existir."""
    atual = os.environ.get('PYTHONPATH', '')
    pasta = get_package_prefix('projeto_bloco')
    for _ in range(3):
        pasta = os.path.dirname(pasta)
        for site in sorted(glob.glob(os.path.join(pasta, '.venv', 'lib', 'python3*',
                                                  'site-packages'))):
            if site not in atual.split(os.pathsep):
                return site + (os.pathsep + atual if atual else '')
            return atual
    return atual


def generate_launch_description():
    share = get_package_share_directory('projeto_bloco')
    desc = get_package_share_directory('projeto_bloco_description')
    params = os.path.join(share, 'config', 'params.yaml')
    navegacao = os.path.join(share, 'config', 'navegacao.yaml')
    urdf = os.path.join(desc, 'urdf', 'robo_sniper.urdf.xacro')
    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch',
                                 'gazebo.launch.py')
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                               'bringup_launch.py')
    sim_time = {'use_sim_time': True}
    show = LaunchConfiguration('show')
    bag = LaunchConfiguration('bag')
    # com a arvore, a patrulha manda os goals para ela; sem a arvore, direto para o Nav2
    acao = PythonExpression(["'navegar_com_pausa' if '", LaunchConfiguration('arvore'),
                             "' == 'true' else 'navigate_to_pose'"])

    argumentos = [
        DeclareLaunchArgument('world', default_value=os.path.join(desc, 'worlds',
                                                                  'sala_patrulha.world'),
                              description='mundo do Gazebo'),
        DeclareLaunchArgument('mapa', default_value=os.path.join(share, 'maps', 'sala_alvos.yaml'),
                              description='mapa estatico (yaml do map_server)'),
        DeclareLaunchArgument('nav_params', default_value=os.path.join(share, 'config',
                                                                       'nav2.yaml'),
                              description='parametros do Nav2'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='janela do Gazebo (gzclient)'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz2'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'rviz', 'navegacao.rviz')),
        DeclareLaunchArgument('percepcao', default_value='true',
                              description='rastreador_yolo + servidor_engajar (tiro nos postos)'),
        DeclareLaunchArgument('arvore', default_value='true',
                              description='behavior tree: pausa de 2 s com obstaculo a 0,5 m '
                                          '(false = a patrulha fala direto com o Nav2)'),
        DeclareLaunchArgument('surpresa', default_value='true',
                              description='caixa que surge na frente do robo durante a patrulha'),
        DeclareLaunchArgument('patrulhar', default_value='true',
                              description='no patrulha percorre os postos (false = goals a mao)'),
        DeclareLaunchArgument('voltas', default_value='1',
                              description='quantas vezes a patrulha percorre a rota'),
        DeclareLaunchArgument('bag', default_value='',
                              description='pasta do ros2 bag record da sessao ("" = nao grava)'),
        DeclareLaunchArgument('csv', default_value='',
                              description='arquivo CSV com uma linha por posto da patrulha'),
        DeclareLaunchArgument('encerrar', default_value='false',
                              description='encerra o launch quando a patrulha termina'),
        DeclareLaunchArgument('show', default_value='false', description='janelas cv2.imshow'),
    ]

    ambiente = [
        SetEnvironmentVariable('PYTHONPATH', pythonpath_com_venv()),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                               os.path.join(share, 'config', 'fastdds_profile.xml')),
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

    # ---- 2. spawn na origem do mundo, que e a origem do mapa (pose inicial do AMCL) ----
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-topic', 'robot_description', '-entity', 'robo_sniper', '-z', '0.02'])

    # ---- 3. depois do spawn ----
    jsp = Node(package='joint_state_publisher', executable='joint_state_publisher',
               parameters=[{'source_list': ['/gazebo/joint_states', '/torre/joint_states'],
                            'rate': 20, **sim_time}])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={'map': LaunchConfiguration('mapa'),
                          'params_file': LaunchConfiguration('nav_params'),
                          'use_sim_time': 'true', 'autostart': 'true', 'slam': 'False',
                          'use_composition': 'False'}.items())
    percepcao = IfCondition(LaunchConfiguration('percepcao'))
    rastreador = Node(package='projeto_bloco', executable='rastreador_yolo', output='screen',
                      parameters=[params, navegacao, {'show': show}], condition=percepcao)
    servidor = Node(package='projeto_bloco', executable='servidor_engajar', output='screen',
                    parameters=[params, navegacao, {'show': show}], condition=percepcao)
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[sim_time],
                condition=IfCondition(LaunchConfiguration('rviz')))
    arvore = Node(package='projeto_bloco', executable='arvore_navegacao', output='screen',
                  parameters=[params, navegacao],
                  condition=IfCondition(LaunchConfiguration('arvore')))
    surpresa = Node(package='projeto_bloco', executable='obstaculo_surpresa', output='screen',
                    parameters=[params, navegacao],
                    condition=IfCondition(LaunchConfiguration('surpresa')))
    patrulha = Node(package='projeto_bloco', executable='patrulha', output='screen',
                    parameters=[params, navegacao,
                                {'acao': acao, 'arquivo_csv': LaunchConfiguration('csv'),
                                 'voltas': LaunchConfiguration('voltas')}],
                    condition=IfCondition(LaunchConfiguration('patrulhar')))

    # ---- 4. gravacao da sessao ----
    gravar = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag] + TOPICOS_BAG, output='screen',
        condition=IfCondition(PythonExpression(["'", bag, "' != ''"])))

    depois_do_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[LogInfo(msg='[navegacao] robo no Gazebo: subindo Nav2, percepcao, RViz, arvore, '
                             'obstaculo surpresa, gravacao e patrulha'),
                 jsp, nav2, rastreador, servidor, rviz, arvore, surpresa, gravar, patrulha]))

    # ---- 5. fim da patrulha encerra tudo (o ros2 bag record fecha o bag ao receber o SIGINT) ----
    fim_da_patrulha = RegisterEventHandler(
        OnProcessExit(target_action=patrulha,
                      on_exit=[LogInfo(msg='[navegacao] patrulha terminou: encerrando o launch'),
                               EmitEvent(event=Shutdown(reason='patrulha concluida'))]),
        condition=IfCondition(LaunchConfiguration('encerrar')))

    return LaunchDescription(argumentos + ambiente + [
        LogInfo(msg='[navegacao] 1/3 Gazebo + robot_state_publisher'),
        gazebo, rsp,
        LogInfo(msg='[navegacao] 2/3 spawn do robo (aguarda o gzserver)'),
        spawn,
        depois_do_spawn,
        fim_da_patrulha,
    ])
