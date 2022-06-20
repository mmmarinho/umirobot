#!/usr/bin/env python
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
import multiprocessing as mp
import multiprocessing.managers as mm

from umirobot import UMIRobot
from umirobot.shared_memory import UMIRobotSharedMemoryProvider


def umirobot_communication_loop(run):
    with UMIRobot() as umirobot, mm.SharedMemoryManager() as smm:
        # Lock
        lock = mp.Lock()
        # Provider
        shared_memory_provider = UMIRobotSharedMemoryProvider(shared_memory_manager=smm, lock=lock)
        # Receiver
        shared_memory_receiver_process = mp.Process(
            target=run,
            args=(shared_memory_provider.get_shared_memory_receiver_initializer_args(), lock)
        )
        shared_memory_receiver_process.start()

        try:
            # Control loop
            while True:

                # Connect to the serial port if requested
                if shared_memory_provider.get_port() is not None:
                    if shared_memory_provider.get_port_connect_signal():
                        if shared_memory_provider.get_port() != umirobot.get_port():
                            umirobot.set_port(shared_memory_provider.get_port())
                            shared_memory_provider.send_port_connect_signal(False)

                # If connection is open, update, handle q and qd
                if umirobot.is_open():
                    umirobot.update()
                    shared_memory_provider.send_q(umirobot.get_q())
                    shared_memory_provider.send_potentiometer_values(umirobot.get_potentiometer_values())
                    umirobot.set_qd(shared_memory_provider.get_qd())

                # Always send connection status
                shared_memory_provider.send_is_open(umirobot.is_open())

                # Check if shutdown signal was sent by receiver
                if shared_memory_provider.get_shutdown_flag():
                    print('main::__main__::Info::Provider shutdown by receiver.')
                    break

        except Exception as e:
            print('main::__main__::Error::' + str(e))
        except KeyboardInterrupt:
            print('main::__main__::Info::Shutdown by CTRL+C.')
            shared_memory_provider.send_shutdown_flag(True)

        # The child process can die before the main process calls join,
        # in which case the main process gets stuck waiting forever.
        if shared_memory_receiver_process.is_alive():
            shared_memory_receiver_process.join()
