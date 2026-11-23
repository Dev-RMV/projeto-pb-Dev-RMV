import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'projeto_bloco'

setup(
    name=package_name,
    version='4.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'modelos'), glob('modelos/*.pt')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmv',
    maintainer_email='rmv@todo.todo',
    description='Robo Sniper: visao TP1; YOLO + Kalman e action de engajamento TP2; bringup '
                'Gazebo + SLAM e segmentacao TP3; navegacao Nav2, patrulha e behavior tree TP4',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'camera_publisher = projeto_bloco.camera_publisher:main',
            'color_segmenter = projeto_bloco.color_segmenter:main',
            'face_features = projeto_bloco.face_features:main',
            'rastreador_yolo = projeto_bloco.rastreador_yolo:main',
            'servidor_engajar = projeto_bloco.servidor_engajar:main',
            'cliente_engajar = projeto_bloco.cliente_engajar:main',
            'segmentador_yolo = projeto_bloco.segmentador_yolo:main',
            'piloto_demo = projeto_bloco.piloto_demo:main',
            'salvar_imagens = projeto_bloco.salvar_imagens:main',
            'patrulha = projeto_bloco.patrulha:main',
            'arvore_navegacao = projeto_bloco.arvore_navegacao:main',
            'obstaculo_surpresa = projeto_bloco.obstaculo_surpresa:main',
            'analise_bag = projeto_bloco.analise_bag:main',
        ],
    },
)
