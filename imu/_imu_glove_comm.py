"""
Copyright (C) 2023 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
import serial
from datetime import datetime
import numpy as np


class IMUGloveComm:
    def __init__(self,
                 baudrate=115200,
                 timeout=0.5):
        self.serial_ = serial.Serial()
        self.serial_.baudrate = baudrate
        self.serial_.timeout = timeout
        self.port = None

        self.raw_accelerometer_values = [None, None, None]
        self.raw_gyrometer_values = [None, None, None]
        self.button = None
        self.frame_count = 0

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self.log("Info::Closing connection at {}.".format(str(self.port)))
        self.serial_.close()

    def set_port(self, port):
        """
        Sets the serial port connection. Automatically (re-)opens the serial communication with the new port.
        :param port: The serial communication port, for instance 'COM3' in Windows.
        """
        self.log("Info::Connecting to {}.".format(port))
        if port is not None:
            if self.is_open():
                self.close()
            self.port = port
            self.serial_.setPort(port)
            self.open()

    def log(self, msg):
        """
        Logs an error message.
        :param msg: The message to be sent.
        """
        date_time_str = datetime.now().strftime("%m/%d/%Y, %H:%M:%S ")
        print(date_time_str + " IMUComm::" + msg)

    def is_open(self):
        """
        Checks if the serial port connection is open.
        :return: True if the serial conncation is open, False otherwise.
        """
        return self.serial_.is_open

    def open(self):
        """
        Opens the serial port communication.
        """
        try:
            if self.serial_.is_open:
                self.log("Warning::open::Port was not closed.")
            else:
                self.serial_.open()
        except Exception as e:
            self.log("Error::open::" + str(e))

    def get_raw_accelerometer_values(self):
        return self.raw_accelerometer_values

    def get_raw_gyrometer_values(self):
        return self.raw_gyrometer_values

    def get_button(self):
        return self.button

    def get_frame_number(self):
        return self.frame_count

    def update(self):
        if self.is_open():
            try:
                serial_bytes = self.serial_.readline()
                # Decodes input line
                serial_string = serial_bytes.decode("ascii")
                splitted_string = serial_string.split(" ")
                splitted_string_size = len(splitted_string)
                if splitted_string_size == 8:
                    if splitted_string[splitted_string_size - 1] == "IMU\r\n":
                        i = 0
                        # Accelerometer
                        for j in range(0, 3):
                            self.raw_accelerometer_values[j] = float(splitted_string[i + j])
                        i = i + 3
                        # Gyrometer
                        for j in range(0, 3):
                            self.raw_gyrometer_values[j] = float(splitted_string[i + j])
                        i = i + 3
                        self.button = True if int(splitted_string[i]) == 1 else False

                        self.frame_count = self.frame_count + 1


            except serial.SerialException as e:
                self.log("Error::update::" + str(e))
                self.log("Info::update::SerialException received, closing connection.")
                self.close()
            except Exception as e:
                self.log("Error::update::" + str(e))


if __name__ == "__main__":
    # Test IMUGloveComm
    with IMUGloveComm() as imu_glove_comm:
        imu_glove_comm.set_port('COM3')
        print("Press CTRL+C to end.")
        raw_value_calibration_sample_count = 0
        a_cal = None
        w_cal = None
        while True:
            try:
                imu_glove_comm.update()
                a = imu_glove_comm.get_raw_accelerometer_values()
                w = imu_glove_comm.get_raw_gyrometer_values()
                b = imu_glove_comm.get_button()
                if b:
                    if a_cal is None:
                        a_cal = np.asarray(a)
                        w_cal = np.asarray(w)
                        raw_value_calibration_sample_count = raw_value_calibration_sample_count + 1
                    else:
                        raw_value_calibration_sample_count = raw_value_calibration_sample_count + 1
                        a_cal = (a_cal * float(raw_value_calibration_sample_count - 1) + np.asarray(a)) / float(
                            raw_value_calibration_sample_count)
                        w_cal = (w_cal * float(raw_value_calibration_sample_count - 1) + np.asarray(w)) / float(
                            raw_value_calibration_sample_count)
                        print(a_cal)
                        print(np.linalg.norm(a_cal))
                        print(w_cal)



            except Exception as e:
                print("imu_glove_comm::Error::" + str(e))
            except KeyboardInterrupt:
                print("imu_glove_comm::Info::Execution ended by user.")
                break
