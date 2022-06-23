"""
Copyright (C) 2020-2022 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
import numpy as np
from dqrobotics.utils.DQ_Math import deg2rad
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH


class UMIRobotCSimRobot:
    def __init__(self, csim_interface):
        self.joint_names = ['UMIRobot_joint_1',
                            'UMIRobot_joint_2',
                            'UMIRobot_joint_3',
                            'UMIRobot_joint_4',
                            'UMIRobot_joint_5']
        self.gripper_name = 'UMIRobot_joint_6'
        self.reference_frame_name = 'UMIRobot_reference_frame'
        self.pose_frame_name = 'x'
        self.desired_pose_frame_name = 'xd'
        self.csim_interface = csim_interface

    def _get_reference_frame_from_csim(self):
        return self.csim_interface.get_object_pose(self.reference_frame_name)

    def get_q_from_csim(self):
        return self.csim_interface.get_joint_positions(self.joint_names)

    def send_q_to_csim(self, q):
        self.csim_interface.set_joint_target_positions(self.joint_names, q)

    def send_gripper_value_to_csim(self, gripper_value):
        self.csim_interface.set_joint_target_position(self.gripper_name, gripper_value)

    def get_gripper_value_from_csim(self):
        return self.csim_interface.get_joint_position(self.gripper_name)

    def kinematics(self):
        dh_matrix = np.matrix([
            deg2rad([0, -90, 0, 0, 0]),
            np.array([24.50, 0, 0, 123.33 + 41.86, 0]) / 1000.0,
            np.array([0, 105.8 - 24.50, 0, 0, 0]) / 1000.0,
            deg2rad([-90, 180, 90, -90, 0]),
            [0, 0, 0, 0, 0]
        ])
        robot = DQ_SerialManipulatorDH(dh_matrix)
        robot.set_reference_frame(self._get_reference_frame_from_csim())
        return robot

    def show_x_in_csim(self, x):
        self.csim_interface.set_object_pose(self.pose_frame_name, x)

    def show_xd_in_csim(self, xd):
        self.csim_interface.set_object_pose(self.desired_pose_frame_name, xd)

    def get_xd_from_csim(self):
        return self.csim_interface.get_object_pose(self.desired_pose_frame_name)
