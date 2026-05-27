import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from robotiq_2f_urcap_adapter.action import GripperCommand


class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander_node')

        self._gripper_client = ActionClient(
            self,
            GripperCommand,
            '/robotiq_2f_urcap_adapter/gripper_command'
        )

        self.get_logger().info('Gripper-only test node started.')
        self.run_sequence()

    def send_gripper(self, pos):
        if not self._gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper server not available!')
            return False

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = pos
        goal_msg.command.max_speed = 0.1
        goal_msg.command.max_effort = 50.0

        self.get_logger().info(f'Sending gripper to {pos}')
        self._gripper_client.send_goal_async(goal_msg)
        time.sleep(1.5)
        return True

    def run_sequence(self):
        print("\n--- SAFETY: Keep hand on E-Stop ---")
        input("Press ENTER to start the gripper-only test...")

        try:
            self.get_logger().info('Step 1: Slightly open gripper')
            if not self.send_gripper(0.025):
                return
            time.sleep(2.0)

            self.get_logger().info('Step 2: Close gripper')
            if not self.send_gripper(0.0):
                return
            time.sleep(2.0)

            self.get_logger().info('Step 3: Open gripper wider')
            if not self.send_gripper(0.085):
                return
            time.sleep(2.0)

            self.get_logger().info('Gripper-only test finished successfully.')

        except KeyboardInterrupt:
            self.get_logger().info('Shutdown requested.')


def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    rclpy.shutdown()


if __name__ == '__main__':
    main()