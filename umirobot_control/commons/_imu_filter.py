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
IMU that has an accelerometer and a gyrometer. The guaternion-based accelerometer filter was NOT presented in [1].
It might not be original, but I did not base it on any existing filter that I know of. It's inspired on
quaternion-based kinematic control.

Adaptive gain reference:
[1] Keeping a Good Attitude: A Quaternion-Based Oritantion Filter for IMUs and MARGs.
Valenti, R.G.; Dryanovski, I.; Xiao, J., Sensors 2015, 15, 19302-19330
"""
import time

import numpy.linalg.linalg
from dqrobotics import *
import numpy as np
from math import cos, sin
from _imu_glove_comm import IMUGloveComm
from dqrobotics.utils.DQ_Math import deg2rad
from dqrobotics.interfaces.vrep import DQ_VrepInterface


class IMUFilter:
    def __init__(self,
                 adaptive_gain_bounds: (float, float) = (0.1, 0.2),
                 accelerometer_weight: float = 0.1,
                 accelerometer_filter_gain: float = 0.001,
                 calibration_required_samples: int = 20,
                 accelerometer_bias=-DQ([9, 16.0, 2]) * 9.81,  # Obtained from
                 # _imu_glove_comm main script
                 gyrometer_bias=-DQ([162.0, 127.0, -147.0])):
        self.calibrated_: bool = False
        self.calibration_required_samples_: int = calibration_required_samples
        self.calibration_valid_sample_count_: int = 0

        self.adaptive_gain_bounds_ = adaptive_gain_bounds
        self.accelerometer_weight_ = accelerometer_weight
        self.accelerometer_filter_gain_ = accelerometer_filter_gain

        self.accelerometer_bias_ = accelerometer_bias
        self.gyrometer_bias_ = gyrometer_bias

        self.absolute_acceleration_ = DQ([0.0])
        self.absolute_angular_velocity_ = DQ([0.0])
        self.current_rotation_ = DQ([1.0])

    def set_absolute_acceleration(self, raw_acceleration: DQ):
        self.absolute_acceleration_ = - (raw_acceleration + self.accelerometer_bias_) * 0.01

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

    def _get_accelerometer_adaptive_gain(self):
        magnitude_error: float = abs(np.linalg.norm(vec4(self.get_absolute_acceleration())) - 9.81) / 9.81
        if magnitude_error < 0.1:
            adaptive_gain = 1.0
        elif magnitude_error > 0.2:
            adaptive_gain = 0.0
        else:
            adaptive_gain = 1.0 - (magnitude_error - 0.1) / 0.1
        return adaptive_gain

    def get_accelerometer_rotation_estimate(self,
                                            T: float):

        def get_J_pinv(r: DQ):
            """
            :param r: The rotation quaternion.
            :return: the pinv of the Jacobian matrix to control that rotation to match -k with gravity.
            """
            return 0.5 * np.array(
                [[0, r.q[2], -r.q[1], -r.q[0]],
                 [0, -r.q[3], -r.q[0], r.q[1]],
                 [0, r.q[0], -r.q[3], r.q[2]],
                 [0, -r.q[1], -r.q[2], -r.q[3]]])

        a: DQ = self.get_absolute_acceleration()
        a_norm: float = np.linalg.norm(vec3(a))

        # Defines how much we can trust that a is an estimate of the gravity
        adaptive_gain: float = self._get_accelerometer_adaptive_gain()

        r_k: DQ = self.get_current_rotation()

        # Measured gravity vector with norm adjustment
        g_y: DQ = a * (1.0 / a_norm)  # The 9.81 will cancel itself out

        # Get error (gravity vectors)
        e: DQ = (conj(r_k) * (-k_) * r_k) - g_y

        # Get the filter signal -> The two following lines are replaced by the closed form given by get_J_pinv
        # Jg = (hamiplus4(conj(r_k) * -k_) + haminus4(-k_ * r_k) @ C4())
        # Jg_inv = numpy.linalg.pinv(Jg)
        r_dot: DQ = adaptive_gain * self.accelerometer_filter_gain_ * DQ(get_J_pinv(r_k) @ vec4(-e))
        w_q: DQ = 2.0 * conj(r_k) * r_dot # Should be pure but sometimes has nontrivial real part
        w = w_q.Im() # TODO Investigate why w is not a pure quaternion as expected
        if T == 0:
            return r_k

        ra: DQ = r_k * exp(w * (1.0 / T))

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

        estimated_gyro_r: DQ = cos(estimated_gyro_angle * T / 2.0) \
                               + estimated_gyro_vector * sin(estimated_gyro_angle * T / 2.0)

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
        self.set_current_rotation(normalize(r_fused)) # The iterative nature of this algorithm makes this one and
        # only normalize eventually necessary.


if __name__ == "__main__":
    # Test IMUGloveComm
    vi = None
    with IMUGloveComm() as imu_glove_comm:
        imu_glove_comm.set_port('COM3')
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
                w = imu_glove_comm.get_raw_gyrometer_values()
                b = imu_glove_comm.get_button()
                frame = imu_glove_comm.get_frame_number()
                if b and None not in a and None not in w and frame > past_frame:
                    imu_filter.set_absolute_acceleration(DQ(a))
                    imu_filter.set_absolute_angular_velocity(DQ(w))
                    T = this_time - last_time
                    imu_filter.update_rotation_estimate(T)
                    r = imu_filter.get_current_rotation()
                    past_frame = frame
                    vi.set_object_rotation("Frame", r)
                last_time = this_time
            except KeyboardInterrupt:
                print("imu_glove_comm::Info::Execution ended by user.")
                break
    if vi is not None:
        vi.stop_simulation()
        vi.disconnect()
