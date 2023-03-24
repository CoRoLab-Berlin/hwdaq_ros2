import numpy as np
import rclpy
from hwdaq import HWDAQ
from rclpy.node import Node

from hwdaq_msgs.msg import HwdaqDesiredPressure, HwdaqOutput


class pneumatic_arm_control(Node):
    hwdaq = HWDAQ()
    data = np.zeros(8, dtype=np.float64)
    control_signal = np.zeros(4, dtype=np.float64)
    des_values = np.zeros(4, dtype=np.float64)
    time_last_des_values = 0

    def __init__(self):
        super().__init__("pneumatic_arm_control")
        self.get_logger().info("pneumatic_arm_control running")
        self.declare_parameter("rate", 100.0)
        self.rate = self.get_parameter("rate").get_parameter_value().double_value

        self.control_loop = self.create_timer(1 / self.rate, self.hardware_rw_spi)

        self.measurments = self.create_publisher(
            HwdaqOutput, "pneumatic_arm_measurments", 10
        )

        self.sub_controller = self.create_subscription(
            HwdaqDesiredPressure,
            "pneumatic_arm_desired_values",
            self.des_values_callback,
            10,
        )
        self.sub_controller
        self.get_logger().info(
            "pneumatic_arm_control ready. running at " + str(self.rate) + " Hz"
        )

    def hardware_rw_spi(self) -> None:
        """
        Main loop for the hardware. Reads the ADC and writes the DAC.
        :return: None
        """
        self.data = self.hwdaq.getADC().copy()

        self.controller()
        self.hwdaq.setDAC(self.control_signal.copy())

        # Publish data for later analysis
        meas = HwdaqOutput()
        meas.header.stamp = self.get_clock().now().to_msg()
        meas.meas_pressure = self.data[:2]
        meas.meas_angle = self.data[4]
        meas.dp_ref = self.control_signal[:2]
        self.measurments.publish(meas)

    def des_values_callback(self, msg: HwdaqDesiredPressure) -> None:
        """
        Callback function for the desired values
        :param msg: Desired values
        :return: None
        """
        self.time_last_des_values = (
            (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            if msg.header.stamp.sec != 0
            else self.get_clock().now().nanoseconds * 1e-9
        )
        self.des_values[:2] = msg.des_pressure

    @property
    def time_ok(self) -> bool:
        """
        Checks if the time of the last desired values message is ok
        :return: True if the time is ok
        """
        time_now = self.get_clock().now().nanoseconds * 1e-9
        return (time_now - self.time_last_des_values) < 0.5

    def controller(self) -> None:
        """
        P control for the pressure
        :return: None
        """
        kp_gain = 0.5
        self.control_signal = kp_gain * (self.des_values - self.data[:4])
        self.control_signal = np.clip(self.control_signal, 0, 10, dtype=np.float64)
        if not self.time_ok:
            self.control_signal = np.ones(4) * 4.5
            self.get_logger().warning(
                "pneumatic_arm_control timing issue, setting to 4.5V",
                throttle_duration_sec=5.0,
            )


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
