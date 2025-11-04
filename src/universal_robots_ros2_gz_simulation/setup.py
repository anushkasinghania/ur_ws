from setuptools import setup
import os

package_name = 'universal_robots_ros2_gz_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name] if os.path.isdir(package_name) else [],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ur10e_gripper_gazebo.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/ur10e_with_dummy_gripper.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anushka',
    maintainer_email='anushkaintern2025@gmail.com',
    description='UR10e + dummy gripper simulation package',
    license='MIT',
    entry_points={},
)

