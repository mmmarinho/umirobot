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
import time

from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from math import pi


def get_control_signal_from_vrep(vi: DQ_VrepInterface):
    r = vi.get_object_rotation("Frame")
    x_dot = rotation_angle(conj(r) * k_) - pi
    y_dot = rotation_angle(conj(r) * j_) - pi
    z_dot = -(rotation_angle(conj(r) * i_) - pi)
    return 0.03 * DQ([x_dot, y_dot, z_dot])


vi = DQ_VrepInterface()
try:
    if not vi.connect("127.0.0.1", 19998, 100, 100):
        vi.disconnect_all()
        raise Exception("Unable to connect to VREP")

    xd_init = vi.get_object_pose("xd")
    xd = xd_init
    T = 0.01
    while True:
        print(get_control_signal_from_vrep(vi))
        if vi.get_object_translation("Button") == i_:
            u = get_control_signal_from_vrep(vi)
            xd = xd * (1 + 0.5 * E_ * (u * T))
            vi.set_object_pose("xd", xd)
        time.sleep(T)
except KeyboardInterrupt:
    print("KeyboardInterrupt")
except Exception as e:
    print("Exception:", e)
vi.disconnect()
