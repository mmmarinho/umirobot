"""
Copyright (C) 2023 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.

This source file is used to filter and estimate the rotation and position of a given
IMU that has an accelerometer and a gyrometer. It is based on some of the content of [1]
below, but with a little bit more solid use of quaternion algebra.

Reference:
[1] Keeping a Good Attitude: A Quaternion-Based Oritantion Filter for IMUs and MARGs.
Valenti, R.G.; Dryanovski, I.; Xiao, J., Sensors 2015, 15, 19302-19330
"""
import time

import numpy.linalg.linalg
from dqrobotics import *
import numpy as np
from math import cos, sin, pi
from _imu_glove_comm import IMUGloveComm
from dqrobotics.utils.DQ_Math import deg2rad
from dqrobotics.interfaces.vrep import DQ_VrepInterface


def _adjust_dq_quadrant(dq):
    # Calculate error from invariant
    error_1 = vec4(conj(rotation(dq)) - 1)
    error_2 = vec4(conj(rotation(dq)) + 1)

    # Calculate 'distance' from invariant
    norm_1 = np.linalg.norm(error_1)
    norm_2 = np.linalg.norm(error_2)

    # Check the closest invariant and return the proper error
    if norm_1 < norm_2:
        return dq
    else:
        print("DQ flipped")
        return -dq


class IMUFilter:
    def __init__(self,
                 adaptive_gain_bounds: (float, float) = (0.1, 0.2),
                 accelerometer_weight: float = 0.01,
                 calibration_required_samples: int = 20,
                 accelerometer_bias=-DQ([9, 16.0, 2]) * 9.81,  # Obtained from
                 # _imu_glove_comm main script
                 gyrometer_bias=-DQ([162.0, 127.0, -147.0])):
        self.calibrated_: bool = False
        self.calibration_required_samples_: int = calibration_required_samples
        self.calibration_valid_sample_count_: int = 0

        self.adaptive_gain_bounds_ = adaptive_gain_bounds
        self.accelerometer_weight_ = accelerometer_weight

        self.accelerometer_bias_ = accelerometer_bias
        self.gyrometer_bias_ = gyrometer_bias

        self.absolute_acceleration_ = DQ([0.0])
        self.absolute_angular_velocity_ = DQ([0.0])
        self.current_rotation_ = DQ([1.0])
        self.current_position_ = DQ([0.0])
        self.current_linear_velocity_ = DQ([0.0])

    def run_calibration_step(self):
        if self._get_accelerometer_adaptive_gain() < 1.0:
            return

        r_now = self.get_current_rotation()
        ra = self.get_accelerometer_rotation_estimate()  # Because adaptive gain is 1.0, fully trust it

        rc: DQ = r_now * exp((self.calibration_valid_sample_count_ / self.calibration_valid_sample_count_)
                             * log(conj(r_now) * ra))  # Get iterative average
        self.calibration_valid_sample_count_ = self.calibration_valid_sample_count_ + 1

        self.set_current_position(rc)
        if self.calibration_valid_sample_count_ >= self.calibration_required_samples_:
            self.calibrated_ = True

    def set_absolute_acceleration(self, raw_acceleration: DQ):
        # The acceleration is the opposite of what we're expecting
        self.absolute_acceleration_ = - (raw_acceleration + self.accelerometer_bias_) * 0.01
        # print("Absolute acceleration {}.".format(self.absolute_acceleration_))
        # print("DQ(a) {}.".format(raw_acceleration))

    def get_absolute_acceleration(self):
        return self.absolute_acceleration_

    def get_accelerometer_weight(self):
        return self.accelerometer_weight_

    def set_absolute_angular_velocity(self, raw_angular_velocity: DQ):
        self.absolute_angular_velocity_ = DQ(deg2rad(vec4(raw_angular_velocity + self.gyrometer_bias_) * 0.01))

    def get_absolute_angular_velocity(self):
        return self.absolute_angular_velocity_

    def set_current_rotation(self, new_rotation):
        self.current_rotation_ = new_rotation

    def get_current_rotation(self):
        return self.current_rotation_

    def get_current_position(self):
        return self.current_position_

    def set_current_position(self, position):
        self.current_position_ = position

    def get_current_linear_velocity(self):
        return self.current_linear_velocity_

    def set_current_linear_velocity(self, velocity):
        self.current_linear_velocity_ = velocity

    def _get_accelerometer_adaptive_gain(self):
        magnitude_error: float = abs(np.linalg.norm(vec4(self.get_absolute_acceleration())) - 9.81) / 9.81
        if magnitude_error < 0.1:
            adaptive_gain = 1.0
        elif magnitude_error > 0.2:
            adaptive_gain = 0.0
        else:
            adaptive_gain = 1.0 - (magnitude_error - 0.1) / 0.1
        return adaptive_gain

    def update_linear_estimates(self,
                                T: float):
        """
        Updates the linear {acceleration, velocity, position} estimates based on the absolute_acceleration
        and the current_rotation.
        """
        adaptive_gain_for_linear: float = 1.0 - self._get_accelerometer_adaptive_gain()
        #adaptive_gain_for_linear: float = 1.0

        # Get the gravity vector estimation based on the current rotation
        r_now: DQ = self.get_current_rotation()
        g_now: DQ = r_now * -9.81 * k_  * conj(r_now)

        # The adaptive gain is the complementary of the one used on the rotation. Alas,
        # we trust on it using the inverse relationship that we used for the rotation
        # print("Absolute acceleration {}, norm = {}.".format(self.get_absolute_acceleration(),
        #                                                    np.linalg.norm(vec4(self.get_absolute_acceleration()))))
        a_world = self.get_absolute_acceleration()
        # print("a_world {}.".format(a_world))
        estimated_acceleration: DQ = adaptive_gain_for_linear * (g_now - a_world
                                                                 )
        # print("Estimated linear acceleration {}, norm = {}.".format(estimated_acceleration,
        #                                                            np.linalg.norm(vec4(estimated_acceleration))))
        self.set_current_linear_velocity(self.get_current_linear_velocity() + estimated_acceleration * T)
        self.set_current_position(self.get_current_position() + self.get_current_linear_velocity() * T)

    def get_accelerometer_rotation_estimate(self,
                                            T: float):
        """
        Gets the estimated rotation after adjusting for the current gravity estimation.
        [A `novel-ish` algorithm that I haven't seen anywhere but haven't looked around that much either]
        :return: the estimated rotation after adjusting for the current gravity estimation.
        """
        a: DQ = self.get_absolute_acceleration()
        a_norm: float = np.linalg.norm(vec3(a))
        # print("a = {}.".format(a))

        # Defines how much we can trust that a is an estimate of the gravity
        adaptive_gain: float = self._get_accelerometer_adaptive_gain()
        # print("Adaptive gain = {}".format(adaptive_gain))

        r_k: DQ = self.get_current_rotation()

        # Measured gravity vector with norm adjustment
        g_y: DQ = a * (1.0 / a_norm)  # The 9.81 will cancel itself out
        # print("gy = {}".format(g_y))

        # Get gravity vector
        g_k: DQ = r_k * g_y * conj(r_k)  # The 9.81 will cancel itself out
        # print("g_k = {}.".format(g_k))

        # Get error (gravity vectors)
        e: DQ = g_k - (-k_)
        # print("e = {}".format(e))
        # print("norm(e) = {}".format(np.linalg.norm(vec3(e))))

        # Get the filter signal
        Jg = (haminus4(g_y * conj(r_k)) + hamiplus4(r_k * g_y) @ C4())
        Jg_inv = numpy.linalg.pinv(Jg)
        # print(Jg_inv)
        r_dot: DQ = 0.001 * DQ(Jg_inv @ vec4(-e))  # Can add a gain here if needed.

        # Use unit quaternion integration
        # r_k_delta: DQ = exp((T * w_k * adaptive_gain) * 0.5)
        # print(norm(r_k_delta))

        # ra: DQ = r_k * r_k_delta
        if T == 0:
            return r_k
        ra: DQ = normalize(r_k + r_dot * (1.0 / T))
        # print(norm(ra))

        return ra

    def get_gyrometer_rotation_estimate(self,
                                        T: float):
        """
        Gets the estimated rotation after integrating with the gyrometer readings
        during the sampling time T.
        :param T: the sampling time.
        :return: the estimated rotation after integrating with the gyrometer readings.
        """
        estimated_gyro_angle: float = np.linalg.norm(vec4(self.get_absolute_angular_velocity()))
        estimated_gyro_vector: DQ = self.get_absolute_angular_velocity() * (1.0 / estimated_gyro_angle)

        estimated_gyro_r: DQ = _adjust_dq_quadrant(cos(estimated_gyro_angle * T / 2.0) \
                                                   + estimated_gyro_vector * sin(estimated_gyro_angle * T / 2.0))

        r_now: DQ = self.get_current_rotation()
        rg: DQ = r_now * estimated_gyro_r

        return rg

    def update_rotation_estimate(self,
                                 T: float):
        """
        Get the weighted average rotation quaternion between the accelerometer rotation
        estimate and the gyro rotation estimate.
        :return: the fused rotation quaternion depending on the accelerometer weight.
        """
        ra: DQ = self.get_accelerometer_rotation_estimate(T)
        rg: DQ = self.get_gyrometer_rotation_estimate(T)
        r_fused: DQ = rg * exp(self.get_accelerometer_weight() * log(conj(rg) * ra))
        self.set_current_rotation(normalize(r_fused))


if __name__ == "__main__":
    # Test IMUGloveComm
    vi = None
    with IMUGloveComm() as imu_glove_comm:
        imu_glove_comm.set_port('COM5')
        imu_filter = IMUFilter()
        print("Press CTRL+C to end.")
        past_frame = 0
        vi = DQ_VrepInterface()
        if not vi.connect("127.0.0.1", 19997, 100, 100):
            vi.disconnect_all()
            raise Exception("Unable to connect to VREP")
        vi.start_simulation()
        last_time = time.time_ns() / 1e9
        while True:
            try:
                this_time = time.time_ns() / 1e9
                imu_glove_comm.update()
                a = imu_glove_comm.get_raw_accelerometer_values()
                # print("a = {}.".format(a))
                w = imu_glove_comm.get_raw_gyrometer_values()
                b = imu_glove_comm.get_button()
                frame = imu_glove_comm.get_frame_number()
                if b and None not in a and None not in w and frame > past_frame:
                    # print("Frame {}.".format(frame))
                    imu_filter.set_absolute_acceleration(DQ(a))
                    imu_filter.set_absolute_angular_velocity(DQ(w))
                    T = this_time - last_time
                    imu_filter.update_rotation_estimate(T)
                    imu_filter.update_linear_estimates(T)
                    r = imu_filter.get_current_rotation()
                    # t = imu_filter.get_current_position()
                    past_frame = frame
                    vi.set_object_rotation("Frame", r)
                    x = rotation_angle(conj(r) * i_)-pi
                    y = rotation_angle(conj(r) * j_)-pi
                    z = rotation_angle(conj(r) * k_)-pi
                    # vi.set_object_translation("Cuboid", DQ([x, y, z]))
                if not b:
                    imu_filter.set_current_linear_velocity(DQ([0]))
                    imu_filter.set_current_position(DQ([0]))
                last_time = this_time
            except KeyboardInterrupt:
                print("imu_glove_comm::Info::Execution ended by user.")
                break
    if vi is not None:
        vi.stop_simulation()
        vi.disconnect()
