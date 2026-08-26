import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 每一项都按扩展名收紧,不用裸 '*'。worlds/ 下就躺着一个
        # my_world.sdf.bak —— 裸 glob 会把它一并装进 share/,
        # 而 .bak 已被 .gitignore 忽略,别人 clone 后根本没有这个文件,
        # 于是"我这儿装出来的内容"和"仓库里的内容"从此不一致。
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qiking',
    maintainer_email='jianzqi123@gmail.com',
    description='Differential-drive robot with a 16-beam 3D LiDAR: URDF, '
                'Gazebo Harmonic simulation, and 2D SLAM with slam_toolbox.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
