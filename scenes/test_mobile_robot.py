import time

from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from math import cos, sin, pi
import numpy as np

adjust_ = ((cos(pi / 2.0) + i_ * sin(pi / 2.0))
           * (cos(pi / 4.0) + j_ * sin(pi / 4.0))) \
          * (1.0 + 0.5 * E_ * -0.1 * k_)


def send_q_to_vrep(q, vi: DQ_VrepInterface):
    x: float = q[0]
    y: float = q[1]
    phi: float = q[2]

    r: DQ = cos(phi / 2.0) + k_ * sin(phi / 2.0)
    p: DQ = x * i_ + y * j_
    pose: DQ = (1 + E_ * 0.5 * p) * r

    vi.set_object_pose("youBot", pose * conj(adjust_))


def get_q_from_vrep(vi: DQ_VrepInterface):
    base_x: DQ = vi.get_object_pose("youBot") * adjust_
    base_t = vec4(translation(base_x))
    base_phi = rotation_angle(rotation(base_x))

    return np.array([base_t[1], base_t[2], base_phi])


def get_control_signal_from_vrep(vi: DQ_VrepInterface):
    r = vi.get_object_rotation("Frame")
    x_dot = rotation_angle(conj(r) * i_) - pi
    y_dot = rotation_angle(conj(r) * j_) - pi
    phi_dot = - (rotation_angle(conj(r) * k_) - pi)
    return np.array([x_dot, y_dot, phi_dot])


vi = DQ_VrepInterface()
try:
    if not vi.connect("127.0.0.1", 19998, 100, 100):
        vi.disconnect_all()
        raise Exception("Unable to connect to VREP")

    q_init = get_q_from_vrep(vi)
    q = q_init
    T = 0.01
    while True:
        print(get_control_signal_from_vrep(vi))
        if vi.get_object_translation("Button") == i_:
            u = get_control_signal_from_vrep(vi)
            q = q + u * T
            send_q_to_vrep(q, vi)
        time.sleep(T)
except KeyboardInterrupt:
    print("KeyboardInterrupt")
except Exception as e:
    print("Exception:", e)
vi.disconnect()
