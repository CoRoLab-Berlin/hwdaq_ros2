import datetime

import h5py
import numpy as np
import rclpy
from hwdaq import HWDAQ
from rclpy.node import Node


class pneumatic_arm_control(Node):
    hwdaq = HWDAQ()
    data = np.zeros(8)
    control_signal = np.zeros(4)

    def __init__(self):
        super().__init__("pneumatic_arm_control")
        self.get_logger().info("pneumatic_arm_control running")
        self.declare_parameter("rate", 100.0)
        self.rate = self.get_parameter("rate").get_parameter_value().double_value

        self.control_loop = self.create_timer(1 / self.rate, self.hardware_rw_spi)

        self._data_complete = self.hwdaq.getADC().copy()
        self._control_signal_complete = self.control_signal.copy()

    @property
    def data_complete(self):
        return self._data_complete

    @data_complete.setter
    def data_complete(self, data):
        np.concatenate((self._data_complete, data), axis=1)

    @data_complete.getter
    def data_complete(self):
        return self._data_complete

    @property
    def control_signal_complete(self):
        return self._control_signal_complete

    @control_signal_complete.setter
    def control_signal_complete(self, control_signal):
        np.concatenate((self._control_signal_complete, control_signal), axis=1)

    @control_signal_complete.getter
    def control_signal_complete(self):
        return self._control_signal_complete

    def hardware_rw_spi(self) -> None:
        """
        Main loop for the hardware. Reads the ADC and writes the DAC.
        :return: None
        """
        self.get_logger().info("pneumatic_arm_control")
        self.data = self.hwdaq.getADC().copy()

        self.hwdaq.setDAC(self.control_signal)

        self.data_complete = self.data

    def save_data(self):
        self.get_logger().info("pneumatic_arm_control saving data")
        now = datetime.datetime.now()
        filename = now.strftime("%Y-%m-%d_%H-%M-%S") + ".h5"
        print(self.data_complete.shape)
        with h5py.File(filename, "w") as f:
            f.create_dataset("data", data=self.data_complete)
            f.create_dataset("control_signal", data=self.control_signal_complete)


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
