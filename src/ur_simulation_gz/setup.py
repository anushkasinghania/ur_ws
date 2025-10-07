from setuptools import setup

package_name = 'ur_simulation_gz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/launch', ['launch/ur10e_gripper_gazebo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anushka',
    maintainer_email='anushkaintern2025@gmail.com',
    description='UR10e + Robotiq 2F gripper Gazebo simulation package',
    license='MIT',
    entry_points={},
)

