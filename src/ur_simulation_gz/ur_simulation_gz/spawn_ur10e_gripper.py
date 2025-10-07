#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

class UR10eGripperSpawner(Node):
    def __init__(self):
        super().__init__('ur10e_gripper_spawner')

        # Get URDF/Xacro paths
        ur_description_pkg = get_package_share_directory('ur_description')
        ur_urdf_path = os.path.join(ur_description_pkg, 'urdf', 'ur10e_robot.urdf.xacro')

        robotiq_pkg = get_package_share_directory('ros2_robotiq_gripper')
        gripper_urdf_path = os.path.join(robotiq_pkg, 'urdf', 'robotiq_2f_85.urdf.xacro')

        # Combine URDFs if needed (or handle separately)
        self.get_logger().info(f"UR URDF path: {ur_urdf_path}")
        self.get_logger().info(f"Gripper URDF path: {gripper_urdf_path}")

        # Wait for spawn service
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        # Read URDF content
        with open(ur_urdf_path, 'r') as f:
            ur_content = f.read()
        with open(gripper_urdf_path, 'r') as f:
            gripper_content = f.read()

        # You may merge the URDF strings or spawn separately
        self.req = SpawnEntity.Request()
        self.req.name = 'ur10e_2f_gripper'
        self.req.xml = ur_content + gripper_content
        self.req.robot_namespace = ''
        self.req.reference_frame = 'world'

        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.spawn_done)

    def spawn_done(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Spawn result: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Failed to spawn: {e}")
        rclpy.shutdown()

def main():
    rclpy.init()
    spawner = UR10eGripperSpawner()
    rclpy.spin(spawner)

if __name__ == '__main__':
    main()

