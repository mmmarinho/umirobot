"""
Copyright (C) 2022 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
import time

from umirobot.shared_memory import UMIRobotSharedMemoryReceiver
from dqrobotics.interfaces.vrep import DQ_VrepInterface as DQ_CoppeliaSimInterface
from umirobot_control.commons import UMIRobotCSimRobot, normalize_potentiometer_values

configuration = {
    "use_real_umirobot": False,
    "umirobot_port": "COM3"
}

def get_qd(potentiometer_values,
           digital_in_values):
    """
    Modify this function to calculate the desired joint values.
    :param potentiometer_values: the potentiometer values obtained from the Arduino (a list of floats between 0.~5.)
    :param digital_in_values: the digital input values obtained from the Arduino (a list of 1s and 0s)
    :return: a list contaning the desired joint values
    """
    return normalize_potentiometer_values(potentiometer_values)


def control_loop(umirobot_smr, cfg):
    # CoppeliaSim Interface
    csim_interface = DQ_CoppeliaSimInterface()

    try:
        # Some info for the user
        print("configuration_space_control::This example only works if your master is correctly connected.")

        # Try to connect to robot
        umirobot_smr.send_port(cfg["umirobot_port"])
        print("configuration_space_control::Trying to connect to umirobot at port={}.".format(cfg["umirobot_port"]))

        # Try to connect to CoppeliaSim
        if not csim_interface.connect(20000, 100, 10):
            # If connection fails, disconnect and throw exception
            csim_interface.disconnect_all()
            raise Exception("Unable to connect to CoppeliaSim.")

        # This object is used to communicate more easily with CoppeliaSim
        umirobot_csim = UMIRobotCSimRobot(csim_interface=csim_interface)

        # Initialize q_init from the CoppeliaSim state
        q_init = umirobot_csim.get_q_from_csim()

        # Some info for the user
        print("configuration_space_control::Ready to start. ")
        print("configuration_space_control::Use CTRL+C to finish cleanly.")

        # Control loop (We're going to control it open loop, because that is how we operate the real robot)
        # Initialize q with its initial value
        qd = q_init
        while not umirobot_smr.get_shutdown_flag():

            # Obtain potentiometer values
            potentiometer_values = umirobot_smr.get_potentiometer_values()
            # Obtain digital input values
            digital_in_values = umirobot_smr.get_digital_in_values()

            # Check potentiometer_values
            if potentiometer_values[0] is None:
                print("configuration_space_control::Potentiometer values invalid. Is your master connected?"
                      " Retrying...")
                time.sleep(1)
                continue

            # Your strategy to calculate the current q, for example
            qd = get_qd(potentiometer_values=potentiometer_values,
                        digital_in_values=digital_in_values)

            # Update real robot if needed
            if cfg["use_real_umirobot"]:
                if umirobot_smr.is_open():
                    # The joint information has to be sent to the robot as a list of integers
                    umirobot_smr.send_qd([int(qd_element) for qd_element in qd])
                else:
                    raise Exception("UMIRobot port not opened at {}.".format(cfg["umirobot_port"]))

            time.sleep(0.008)

    except Exception as e:
        print("configuration_space_control::control_loop::Exception caught: ", e)
        print("task_space_control::Be sure that your CoppeliaSim scene is correctly loaded and "
              "that the simulation has been started.")
    except KeyboardInterrupt:
        print("configuration_space_control::control_loop::KeyboardInterrupt")
        umirobot_smr.lock.acquire(False)
        umirobot_smr.lock.release()

    # Disconnect from CoppeliaSim
    csim_interface.disconnect()


def run(shared_memory_info, lock):
    umirobot_smr = UMIRobotSharedMemoryReceiver(shared_memory_info, lock)

    try:
        control_loop(umirobot_smr, cfg=configuration)
    except Exception as e:
        print("configuration_space_control::run::Error::" + str(e))
    except KeyboardInterrupt:
        print("configuration_space_control::run::Info::Interrupted by user.")

    umirobot_smr.send_shutdown_flag(True)
