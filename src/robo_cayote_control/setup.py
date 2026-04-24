from setuptools import find_packages, setup

package_name = 'robo_cayote_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mqtt_params.yaml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/mqtt_ack.launch.py',
                'launch/ris_go2rtc.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='palandeer',
    maintainer_email='palandeer@todo.todo',
    description='MQTT command handler for Robo Cayote',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_ack_node = robo_cayote_control.mqtt_ack_node:main',
            'ris_go2rtc_node = robo_cayote_control.ris_go2rtc_node:main',
            'cayote_rl_brain = robo_cayote_control.cayote_rl_brain:main',
            'cayote_rl_brain_v2 = robo_cayote_control.cayote_rl_brain_v2:main',
            'arduino_motor_driver = robo_cayote_control.arduino_motor_driver:main',
            'yolo_processor = robo_cayote_control.yolo_processor:main',
            'mission_controller = robo_cayote_control.mission_controller:main'
        ],
    },
)
