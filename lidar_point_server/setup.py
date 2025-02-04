import os
from setuptools import setup
from glob import glob

package_name = 'lidar_point_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # 또는 find_packages()
    data_files=[
        # Launch 파일을 설치할 경로 지정
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 여기서 'share/<패키지명>/launch' 디렉토리에 launch 파일을 복사
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Example Python package for ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # ros2 run <package> <executable> 형태로 실행할 스크립트 등록
            'save_points = lidar_point_server.save_points:main',
        ],
    },
)
