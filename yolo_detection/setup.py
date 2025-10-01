from setuptools import find_packages, setup

package_name = 'yolo_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/weights', ['weights/best.pt', 'weights/apple.pt']),
        ('share/' + package_name + '/config', ['config/detection.yaml']),
        ('share/' + package_name + '/launch', ['launch/detection.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/camera.rviz']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'message_filters',
    ],
    zip_safe=True,
    maintainer='nilton',
    maintainer_email='nilton.csv18@edu.udesc.br',
    description='Object detection and localization package using YOLO',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = yolo_detection.detection_node:main',
        ],
    },
)
