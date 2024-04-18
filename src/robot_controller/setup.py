from setuptools import find_packages, setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='beatrix',
    maintainer_email='beatrix@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = robot_controller.my_first_node:main",
            "draw_circle = robot_controller.draw_circle:main",
            "pose_subsciber = robot_controller.pose_subscriber:main",
            "turtle_controller = robot_controller.turtle_controller:main",
            "camera_control = robot_controller.camera:main",
            "face_controller = robot_controller.face_command_subscriber:main",
            "gui = robot_controller.gui:main",
            "sound = robot_controller.audio:main",
            "speech = robot_controller.speech:main" 
        ],
    },
)
