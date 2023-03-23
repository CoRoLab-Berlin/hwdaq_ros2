import datetime

import h5py
import numpy as np
import rclpy
from hwdaq import HWDAQ
from rclpy.node import Node

from hwdaq_msgs.msg import HwdaqDesiredPressure, HwdaqMeasurements


class pneumatic_arm_control(Node):
    hwdaq = HWDAQ()
    data = np.zeros(8)
    control_signal = np.zeros(4)
    des_values = np.zeros(4)

    def __init__(self):
        super().__init__("pneumatic_arm_control")
        self.get_logger().info("pneumatic_arm_control running")
        self.declare_parameter("rate", 100.0)
        self.rate = self.get_parameter("rate").get_parameter_value().double_value

        self.control_loop = self.create_timer(1 / self.rate, self.hardware_rw_spi)

        self.measurments = self.create_publisher(
            HwdaqMeasurements, "pneumatic_arm_measurments", 10
        )

        self.sub_controller = self.create_subscription(
            HwdaqDesiredPressure,
            "pneumatic_arm_desired_values",
            self.des_values_callback,
            10,
        )
        self.sub_controller
        self.time_last_des_values = 0

        self._data_complete = self.hwdaq.getADC().copy()
        self._control_signal_complete = self.control_signal.copy()

    def hardware_rw_spi(self) -> None:
        """
        Main loop for the hardware. Reads the ADC and writes the DAC.
        :return: None
        """
        self.get_logger().info("pneumatic_arm_control")
        self.data = self.hwdaq.getADC().copy()

        if not self.time_ok:
            self.des_values = np.zeros(4)

        self.controller()
        self.hwdaq.setDAC(self.control_signal.copy())

        meas = HwdaqMeasurements()
        meas.header.stamp = self.get_clock().now().to_msg()
        meas.data = self.data.copy()
        self.measurments.publish(meas)

    def des_values_callback(self, msg: HwdaqDesiredPressure) -> None:
        """
        Callback function for the desired values
        :param msg: Desired values
        :return: None
        """
        self.get_logger().info("pneumatic_arm_control des_values_callback")
        # exat time of the message
        self.time_last_des_values = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        self.des_values = msg.des_pressure

    @property
    def time_ok(self) -> bool:
        """
        Checks if the time of the last desired values message is ok
        :return: True if the time is ok
        """
        time = self.get_clock().now().seconds_nanoseconds
        return (
            time[0] + time[1] * 1e-9 - self.time_last_des_values
        ).total_seconds() < 0.5

    def controller(self) -> None:
        """
        P control for the pressure
        :return: None
        """
        kp = 0.1
        self.control_signal = kp * (self.des_values - self.data)

    def save_data(self):
        self.get_logger().info("pneumatic_arm_control saving data")
        # now = datetime.datetime.now()
        # filename = now.strftime("%Y-%m-%d_%H-%M-%S") + ".h5"
        # print(self.data_complete.shape)
        # with h5py.File(filename, "w") as f:
        #     f.create_dataset("data", data=self.data_complete)
        #     f.create_dataset("control_signal", data=self.control_signal_complete)


def main():
    rclpy.init()
    controller = pneumatic_arm_control()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.save_data()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
