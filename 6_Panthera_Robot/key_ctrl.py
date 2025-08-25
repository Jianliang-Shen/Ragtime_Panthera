import mujoco
import mujoco.viewer
import numpy as np
import threading
import time
from pynput import keyboard

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
    "1": (0, +1), "q": (0, -1),  # joint1
    "2": (1, +1), "w": (1, -1),  # joint2
    "3": (2, +1), "e": (2, -1),  # joint3
    "4": (3, +1), "r": (3, -1),  # joint4
    "5": (4, +1), "t": (4, -1),  # joint5
    "6": (5, +1), "y": (5, -1),  # joint6
    "7": (6, +1), "u": (6, -1),  # gripper_1_joint
    "8": (7, -1), "i": (7, +1),  # gripper_2_joint
}

joint_step_size = 0.01
gripper_step_size = 0.002
pressed_keys = set()

# 目标检测的 link 名字
monitor_links = {"link1", "link2", "link3","link4", "link5", "link6", "gripper_1_link", "gripper_2_link", "gripper_centor_link"}
monitor_link_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) 
                    for name in monitor_links]

# 记录关节方向的“禁止表” {joint_id: direction}
blocked_dirs = {}

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
        moved = False
        collided_joint = None
        collided_dir = None
        pressed_keys_1 = pressed_keys.copy()
        if pressed_keys_1:
            for k in pressed_keys_1:
                if k in key_map:
                    jid, direction = key_map[k]

                    # print(blocked_dirs)
                    if blocked_dirs.get(jid) == direction:
                        continue
                    else:
                        blocked_dirs = {}

                    step = joint_step_size if jid < 6 else gripper_step_size
                    q_des[jid] += direction * step
                    moved = True
                    current_jid = jid
                    current_dir = direction

        data.qpos[:len(q_des)] = q_des
        data.qvel[:] = 0

        T = np.eye(4)

        print("Joint origins relative to base_link:")
        for i, (params, q) in enumerate(zip(MDH_params, data.qpos[:6]), start=1):
            theta0, d, a, alpha = params
            theta = theta0 + q   # 加上关节变量
            A = mdh_transform(theta, d, a, alpha)
            T = T @ A
            pos = T[:3, 3]
            print(f"Joint {i}: x={pos[0]: .6f}, y={pos[1]: .6f}, z={pos[2]: .6f}")

        mujoco.mj_step(model, data)
        if moved:
            mujoco.mj_forward(model, data)  # 更新碰撞信息
            for i in range(data.ncon):
                contact = data.contact[i]
                b1 = model.geom_bodyid[contact.geom1]
                b2 = model.geom_bodyid[contact.geom2]


                # 计算接触距离
                dist = contact.dist

                if b1 in monitor_link_ids or b2 in monitor_link_ids:
                    # 只有和 world 接触时需要判定 dist<0
                    if b1 == 0 or b2 == 0:  # 0号 body = world
                        if dist >= 0:
                            continue  # 忽略 resting contact
                    # print(b1, b2)
                    if (b1 == 8 and b2 == 7) or (b1 == 7 and b2 == 8):
                        continue

                    name1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b1)
                    name2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b2)
                    print(f"[Collision] link involved: {name1} <-> {name2}, dist={dist}")

                    # 禁止该关节往当前方向继续运动
                    blocked_dirs[current_jid] = current_dir
        viewer.sync()
        time.sleep(0.02)
