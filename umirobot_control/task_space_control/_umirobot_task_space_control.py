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
import numpy as np

from dqrobotics import *
from math import sin, cos
from umirobot.shared_memory import UMIRobotSharedMemoryReceiver
from umirobot_control.task_space_control._umirobot_task_space_controller import UMIRobotTaskSpaceController
from dqrobotics.interfaces.vrep import DQ_VrepInterface as DQ_CoppeliaSimInterface
from dqrobotics.utils.DQ_Math import rad2deg, deg2rad
from umirobot_control.commons import UMIRobotCSimRobot, normalize_potentiometer_values

configuration = {
    "controller_gain": 4.0,
    "damping": 0.01,
    "alpha": 0.999,  # Soft priority between translation and rotation [0,1] ~1 Translation, ~0 Rotation
    "use_real_master": False,
    "use_real_umirobot": True,
    "umirobot_port": "COM4"
}


def get_gripper_value_from_real_master(potentiometer_values):
    """
    Template for the students to implement their own gripper control logic using thei masters.
    :param potentiometer_values: the list of potentiometer values
    :return: A value between -90 and 90 indicating the servo rotation.
    """
    # One example:
    normalized_potentiometer_values = normalize_potentiometer_values(potentiometer_values)
    return normalized_potentiometer_values[5] * deg2rad(90)


def get_xd_from_real_master(potentiometer_values, x_init):
    """
    Template for the students to implement their own task-space control logic using their masters.
    The signals from the potentiometers must be converted into a task-space pose reference for
    the pose controller.
    :param potentiometer_values: the list of potentiometer values
    :param x_init: the initial pose of the robot
    :return: the xd representing the desired position that will be sent to the robot controller
    """
    # One way of doing it
    normalized_potentiometer_values = normalize_potentiometer_values(potentiometer_values)
    tx = normalized_potentiometer_values[0] * 0.02 * i_  # +- 2 cm motion about x
    ty = normalized_potentiometer_values[1] * 0.02 * j_  # +- 2 cm motion about y
    tz = normalized_potentiometer_values[2] * 0.02 * k_  # +- 2 cm motion about z
    phix = normalized_potentiometer_values[3] * deg2rad(90)  # +- 90 deg about x
    rx = cos(phix / 2.0) + i_ * sin(phix / 2.0)
    phiy = normalized_potentiometer_values[4] * deg2rad(90)  # +- 90 deg about x
    ry = cos(phiy / 2.0) + j_ * sin(phiy / 2.0)
    phiz = normalized_potentiometer_values[5] * deg2rad(90)  # +- 90 deg about x
    rz = cos(phiz / 2.0) + k_ * sin(phiz / 2.0)
    # Relative pose
    t = tx + ty + tz
    r = rx * ry * rz
    xd_relative = r + 0.5 * E_ * t * r
    # Absolute pose
    return x_init * xd_relative


def control_loop(umirobot_smr, cfg):
    # Instantiate a CoppeliaSim Interface
    csim_interface = DQ_CoppeliaSimInterface()

    try:
        # Try to connect to robot
        if cfg["use_real_master"] or cfg["use_real_umirobot"]:
            umirobot_smr.send_port(cfg["umirobot_port"])
            print("main::Trying to connect to umirobot at port={}.".format(cfg["umirobot_port"]))

        # Try to connect to VREP
        if not csim_interface.connect(20000, 100, 10):
            # If connection fails, disconnect and throw exception
            csim_interface.disconnect_all()
            raise Exception("Unable to connect to CoppeliaSim.")

        # This object is used to communicate more easily with VREP
        umirobot_csim = UMIRobotCSimRobot(csim_interface=csim_interface)

        # This is a DQ_SerialManipulatorDH instance. Used to calculate kinematics of the UMIRobot.
        umirobot_kinematics = umirobot_csim.kinematics()

        # UMIRobot Task Space Controler
        umirobot_controller = UMIRobotTaskSpaceController(kinematics=umirobot_kinematics)
        umirobot_controller.set_gain(cfg["controller_gain"])
        umirobot_controller.set_damping(cfg["damping"])
        umirobot_controller.set_alpha(cfg["alpha"])

        # Initialize the objects in VREP to reflect what we calculate using DQRobotics
        q_init = umirobot_csim.get_q_from_csim()
        x_init = umirobot_kinematics.fkm(q_init)
        umirobot_csim.show_x_in_csim(x_init)
        umirobot_csim.show_xd_in_csim(x_init)

        # Loop parameters
        sampling_time = 0.008

        # Some info for the user
        print("task_space_control::Ready to start. Move the `xd` on the CoppeliaSim scene.")
        print("task_space_control::Use CTRL+C to finish cleanly.")

        # Control loop (We're going to control it open loop, because that is how we operate the real robot)
        # Initialize q with its initial value
        q = q_init
        while not umirobot_smr.get_shutdown_flag():
            # Get xd from CoppeliaSim
            xd = umirobot_csim.get_xd_from_csim()
            # Get the desired gripper value from CoppeliaSim
            gripper_value_d = umirobot_csim.get_gripper_value_from_csim()

            # Alternatively, you can do it like so:
            # potentiometer_values = umirobot_smr.get_potentiometer_values()
            # xd = get_xd_from_real_master(potentiometer_values,x_init)
            # gripper_value_d = get_gripper_value_from_real_master(potentiometer_values)

            # Solve the quadratic program
            u = umirobot_controller.compute_setpoint_control_signal(q, xd)

            # Update the current joint positions
            q = q + u * sampling_time

            # Update vrep with the new information we have
            umirobot_csim.send_q_to_csim(q)
            umirobot_csim.show_x_in_csim(umirobot_controller.get_last_robot_pose())

            # Update real robot if needed
            if cfg["use_real_umirobot"]:
                if umirobot_smr.is_open():
                    # Control the gripper somehow
                    q_temp = np.hstack((q, gripper_value_d))
                    # The joint information has to be sent to the robot as a list of integers
                    umirobot_smr.send_qd(rad2deg(q_temp).astype(int).tolist())
                else:
                    raise Exception("UMIRobot port not opened at {}.".format(cfg["umirobot_port"]))

    except Exception as e:
        print("task_space_control::control_loop::Exception caught: ", e)
        print("task_space_control::Be sure that your CoppeliaSim scene is correctly loaded and "
              "that the simulation has been started.")
    except KeyboardInterrupt:
        print("task_space_control::control_loop::KeyboardInterrupt")
        umirobot_smr.lock.acquire(False)
        umirobot_smr.lock.release()

    # Disconnect from VREP
    csim_interface.disconnect()


def run(shared_memory_info, lock):
    umirobot_smr = UMIRobotSharedMemoryReceiver(shared_memory_info, lock)

    try:
        control_loop(umirobot_smr, cfg=configuration)
        print("task_space_control::run::Control loop ended.")
    except Exception as e:
        print("task_space_control::run::Error::" + str(e))
    except KeyboardInterrupt:
        print("task_space_control::run::Info::Interrupted by user.")

    umirobot_smr.send_shutdown_flag(True)
