import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'projeto_bloco'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmv',
    maintainer_email='rmv@todo.todo',
    description='Publisher de camera/video, segmentacao HSV, deteccao de rostos e features',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'camera_publisher = projeto_bloco.camera_publisher:main',
            'color_segmenter = projeto_bloco.color_segmenter:main',
            'face_features = projeto_bloco.face_features:main',
        ],
    },
)
