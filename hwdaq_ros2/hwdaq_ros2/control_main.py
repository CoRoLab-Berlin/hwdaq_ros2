import hwdaq
import rclpy
from rclpy.node import Node


def pneumatic_arm_control(Node):
    def __init__(self):
        super().__init__("pneumatic_arm_control")
        self.get_logger().info("pneumatic_arm_control")


def main():
    rclpy.init()
    pneumatic_arm_control()
    try:
        rclpy.spin(pneumatic_arm_control)
    except KeyboardInterrupt:
        pass
    pneumatic_arm_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
