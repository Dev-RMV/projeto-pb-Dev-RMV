from setuptools import find_packages, setup

package_name = 'tp4_extras'

setup(
    name=package_name,
    version='4.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmv',
    maintainer_email='rmv@todo.todo',
    description='TP4: faixas e formas no MetaDrive (OpenCV) e CNN de estados da via (Keras)',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'detectar_faixas = tp4_extras.deteccao_faixas:main',
            'detectar_formas = tp4_extras.deteccao_formas:main',
            'coletar_dataset = tp4_extras.coletar_dataset:main',
            'treinar_cnn = tp4_extras.treinar_cnn:main',
        ],
    },
)
