import hwdaq
import rclpy
from rclpy.node import Node


class pneumatic_arm_control(Node):
    def __init__(self):
        super().__init__("pneumatic_arm_control")
        self.get_logger().info("pneumatic_arm_control")


def main():
    rclpy.init()
    controller = pneumatic_arm_control()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
