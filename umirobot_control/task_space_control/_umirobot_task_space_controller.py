"""
Copyright (C) 2020 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
from dqrobotics.robot_modeling import DQ_Kinematics
from dqrobotics import *
from dqrobotics.solvers import DQ_QuadprogSolver

import numpy as np


class UMIRobotTaskSpaceController:

    def __init__(self, kinematics):
        self.qp_solver = DQ_QuadprogSolver()
        self.kinematics = kinematics
        self.gain = None
        self.damping = None
        self.alpha = None
        self.last_x = None

    def is_set(self):
        if self.gain is not None \
                and self.damping is not None \
                and self.alpha is not None:
            return True
        else:
            return False

    def set_gain(self, gain):
        if gain > 0.:
            self.gain = gain
        else:
            raise Exception("UMIRobotTaskSpaceController::set_gain::Gain should be larger than 0.")

    def set_damping(self, damping):
        if damping > 0.:
            self.damping = damping
        else:
            raise Exception("UMIRobotTaskSpaceController::set_damping::Damping should be larger than 0.")

    def set_alpha(self, alpha):
        if 0 <= alpha <= 1:
            self.alpha = alpha
        else:
            raise Exception(
                "UMIRobotTaskSpaceController::set_alpha::Alpha should be more or equal to zero and less or equal to "
                "one.")

    def get_last_robot_pose(self):
        return self.last_x

    def compute_setpoint_control_signal(self, q, xd):
        """
        Calculate the control step according to
         "A Unified Framework for the Teleoperation of Surgical Robots in Constrained Workspaces".
         Marinho, M. M; et al.
         In 2019 IEEE International Conference on Robotics and Automation (ICRA), pages 2721–2727, May 2019. IEEE
         http://doi.org/10.1109/ICRA.2019.8794363
        :param q: the current joint positions.
        :param xd: the desired pose.
        :return: The desired joint positions that should be sent to the robot.
        """
        if not is_unit(xd):
            raise Exception("UMIRobotTaskSpaceController::compute_setpoint_control_signal::xd should be an unit dual "
                            "quaternion")
        if not self.is_set():
            raise Exception("UMIRobotTaskSpaceController::needs to be initialized with set_gain, set_damping, "
                            "and set_alpha")

        DOF = len(q)

        # Get current pose information
        x = self.kinematics.fkm(q)
        self.last_x = x

        # Calculate errors
        et = vec4(translation(x) - translation(xd))
        er = UMIRobotTaskSpaceController._get_rotation_error(x, xd)

        # Get the Translation Jacobian and Rotation Jacobian
        Jx = self.kinematics.pose_jacobian(q)
        rd = rotation(xd)
        Jr = DQ_Kinematics.rotation_jacobian(Jx)
        Nr = haminus4(rd) @ C4() @ Jr

        Jt = DQ_Kinematics.translation_jacobian(Jx, x)

        # Translation term
        Ht = Jt.transpose() @ Jt
        ft = self.gain * Jt.transpose() @ et

        # Rotation term
        Hr = Nr.transpose() @ Nr
        fr = self.gain * Nr.transpose() @ er

        # Damping term
        Hd = np.eye(DOF, DOF) * self.damping * self.damping

        # Combine terms using the soft priority
        H = self.alpha * Ht + (1.0 - self.alpha) * Hr + Hd
        f = self.alpha * ft + (1.0 - self.alpha) * fr

        # Joint (position) limit constraints +- 90 degrees
        lower_joint_limits = -np.pi / 2.0 * (np.ones((DOF,)))
        upper_joint_limits = np.pi / 2.0 * (np.ones((DOF,)))
        W_jl = np.vstack((-1.0 * np.eye(DOF, DOF), np.eye(DOF, DOF)))
        w_jl = np.hstack((-1.0 * (lower_joint_limits - q), 1.0 * (upper_joint_limits - q)))

        # Solve the quadratic program
        u = self.qp_solver.solve_quadratic_program(H, f, W_jl, w_jl, None, None)

        return u

    @staticmethod
    def _get_rotation_error(x, xd):
        # Calculate error from invariant
        error_1 = vec4(conj(rotation(x))*rotation(xd) - 1)
        error_2 = vec4(conj(rotation(x))*rotation(xd) + 1)

        # Calculate 'distance' from invariant
        norm_1 = np.linalg.norm(error_1)
        norm_2 = np.linalg.norm(error_2)

        # Check the closest invariant and return the proper error
        if norm_1 < norm_2:
            return error_1
        else:
            return error_2
