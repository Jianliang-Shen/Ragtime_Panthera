import mujoco
import mujoco.viewer
import numpy as np
import threading
import time
from pynput import keyboard
from scipy.optimize import least_squares

# 读取机器人模型
model = mujoco.MjModel.from_xml_path("./Panthera_Robot.xml")
data = mujoco.MjData(model)

q_des = np.copy(data.qpos)

# 初始位置拉高，避免初始碰撞
q_des[1] = np.deg2rad(90)   # joint1
q_des[2] = np.deg2rad(90)   # joint2

# MDH参数: theta0, d, a, alpha
MDH_params = [
    [np.pi,         0.105,          0.0,        0.0],       # Joint 1
    [0.0,           0.0,            0.0,        np.pi/2],   # Joint 2
    [-2.83491633,   0.0,            0.18,       -np.pi],    # Joint 3
    [-2.83491266,   -0.0,           0.18880943, np.pi],     # Joint 4
    [-np.pi/2,      -0.00000016,    -0.08,      np.pi/2],   # Joint 5
    [0.0,           0.036,          -0.0,       np.pi/2],   # Joint 6
]

def mdh_transform(theta, d, a, alpha):
    """生成MDH齐次变换矩阵"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -d*sa],
        [st*sa, ct*sa, ca, d*ca],
        [0, 0, 0, 1]
    ])

# 键盘控制映射
key_map = {
    "1": (0, +1), "q": (0, -1),  # forward/backward
    "2": (1, +1), "w": (1, -1),  # left/right
    "3": (2, +1), "e": (2, -1),  # up/down
}

s_step_size = 0.001

pressed_keys = set()

def forward_kinematics(qpos):
    T = np.eye(4)
    for (theta0, d, a, alpha), q in zip(MDH_params, qpos):
        T = T @ mdh_transform(theta0 + q, d, a, alpha)
    return T[:3, 3]   # 返回末端位置

# --------- 目标末端位置 ----------
target = np.array([0.296, 0.0, 0.342])

# --------- 误差函数 ----------
def error_func(qpos):
    pos = forward_kinematics(qpos)
    return pos - target



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
        pressed_keys_1 = pressed_keys.copy()
        if pressed_keys_1:
            for k in pressed_keys_1:
                if k in key_map:
                    i, direction = key_map[k]
                    target[i] += direction * s_step_size

        # --------- 初始猜测 ----------
        q0 = np.zeros(6)

        res = least_squares(error_func, q0, method="trf")
        print("求解到的关节角 qpos =")
        print(res.x)
        print("验证末端位置 =", forward_kinematics(res.x))

        data.qpos[:6] = res.x
        data.qvel[:] = 0


        mujoco.mj_step(model, data)

        viewer.sync()
        time.sleep(0.02)
