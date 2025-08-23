import mujoco
import mujoco.viewer
import numpy as np
import threading
import time
from pynput import keyboard
import os

os.environ["MUJOCO_GL"] = "egl"

model = mujoco.MjModel.from_xml_path("./Panthera_Robot.xml")
data = mujoco.MjData(model)

q_des = np.copy(data.qpos)

key_map = {
    "1": (0, +1), "!": (0, -1),  # joint1
    "2": (1, +1), "@": (1, -1),  # joint2
    "3": (2, +1), "#": (2, -1),  # joint3
    "4": (3, +1), "$": (3, -1),  # joint4
    "5": (4, +1), "%": (4, -1),  # joint5
    "6": (5, +1), "^": (5, -1),  # joint6
    "7": (6, +1), "&": (6, -1),  # gripper_1_joint
    "8": (7, -1), "*": (7, +1),  # gripper_2_joint
}

joint_step_size = 0.1
gripper_step_size = 0.002
pressed_keys = set()

def on_press(key):
    try:
        if key.char in key_map:
            pressed_keys.add(key.char)
    except AttributeError:
        pass

def on_release(key):
    try:
        if key.char in pressed_keys:
            pressed_keys.remove(key.char)
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        if pressed_keys:
            for k in pressed_keys:
                if k in key_map:
                    jid, direction = key_map[k]
                    step = joint_step_size if jid < 6 else gripper_step_size
                    q_des[jid] += direction * step
        
        data.qpos[:len(q_des)] = q_des
        data.qvel[:] = 0

        mujoco.mj_step(model, data)

        viewer.sync()

        time.sleep(0.01)
