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

from umirobot.shared_memory import UMIRobotSharedMemoryReceiver
from umirobot_task_space_controller import UMIRobotTaskSpaceController
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.utils.DQ_Math import rad2deg
from umirobot_vrep_robot import UMIRobotVrepRobot

configuration = {
    "controller_gain": 4.0,
    "damping": 0.01,
    "alpha": 0.999,  # Soft priority between translation and rotation [0,1] ~1 Translation, ~0 Rotation
    "use_real_umirobot": False,
    "umirobot_port": "COM3"
}


def control_loop(umirobot_smr, cfg):
    # Instantiate a DQ_VrepInterface
    vrep_interface = DQ_VrepInterface()

    try:
        # Try to connect to robot
        if cfg["use_real_umirobot"]:
            umirobot_smr.send_port(cfg["umirobot_port"])
            print("main::Trying to connect to umirobot at port={}.".format(cfg["umirobot_port"]))

        # Try to connect to VREP
        if not vrep_interface.connect(20000, 100, 10):
            # If connection fails, disconnect and throw exception
            vrep_interface.disconnect_all()
            raise Exception("Unable to connect to VREP.")

        # This object is used to communicate more easily with VREP
        umirobot_vrep = UMIRobotVrepRobot(vrep_interface=vrep_interface)

        # This is a DQ_SerialManipulatorDH instance. Used to calculate kinematics of the UMIRobot.
        umirobot_kinematics = umirobot_vrep.kinematics()

        # UMIRobot Task Space Controler
        umirobot_controller = UMIRobotTaskSpaceController(kinematics=umirobot_kinematics)
        umirobot_controller.set_gain(cfg["controller_gain"])
        umirobot_controller.set_damping(cfg["damping"])
        umirobot_controller.set_alpha(cfg["alpha"])

        # Initialize the objects in VREP to reflect what we calculate using DQRobotics
        q_init = umirobot_vrep.get_q_from_vrep()
        x_init = umirobot_kinematics.fkm(q_init)
        umirobot_vrep.show_x_in_vrep(x_init)
        umirobot_vrep.show_xd_in_vrep(x_init)

        # Loop parameters
        sampling_time = 0.008

        # Control loop (We're going to control it open loop, because that is how we operate the real robot)
        # Initialize q with its initial value
        q = q_init
        while not umirobot_smr.get_shutdown_flag():
            # Change how you calculate xd
            xd = umirobot_vrep.get_xd_from_vrep()

            # Solve the quadratic program
            u = umirobot_controller.compute_setpoint_control_signal(q, xd)

            # Update the current joint positions
            q = q + u * sampling_time

            # Update vrep with the new information we have
            umirobot_vrep.send_q_to_vrep(q)
            umirobot_vrep.show_x_in_vrep(umirobot_controller.get_last_robot_pose())

             # Update real robot if needed
            if cfg["use_real_umirobot"]:
                if umirobot_smr.is_open():
                    # Get the desired gripper value from VREP
                    gripper_value_d = umirobot_vrep.get_gripper_value_from_vrep()
                    # Control the gripper somehow
                    q_temp = np.hstack((q, gripper_value_d))
                    # The joint information has to be sent to the robot as a list of integers
                    umirobot_smr.send_qd(rad2deg(q_temp).astype(int).tolist())
                else:
                    raise Exception("UMIRobot port not opened at {}.".format(cfg["umirobot_port"]))

    except Exception as e:
        print("umirobot_task_space_control::control_loop::KeyboardInterrupt::Exception caught: ", e)
    except KeyboardInterrupt:
        print("umirobot_task_space_control::control_loop::KeyboardInterrupt")

    # Disconnect from VREP
    vrep_interface.disconnect()


def run(shared_memory_info, lock):
    umirobot_smr = UMIRobotSharedMemoryReceiver(shared_memory_info, lock)

    try:
        control_loop(umirobot_smr, cfg=configuration)
    except Exception as e:
        print("umirobot_task_space_control::run::Error::" + str(e))
    except KeyboardInterrupt:
        print("umirobot_task_space_control::run::Info::Interrupted by user.")

    umirobot_smr.send_shutdown_flag(True)
