from setuptools import setup

package_name = 'pacote_minimo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/exemplo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='aluno@exemplo.com',
    description='Exemplo mínimo com boas práticas',
    license='MIT',
    entry_points={
        'console_scripts': [
            # boas práticas: nome_do_executavel = pacote.modulo:funcao
            'no_publisher = pacote_minimo.no_publisher:main',
            'no_subscriber = pacote_minimo.no_subscriber:main',
        ],
    },
)
