import numpy as np
from scipy.optimize import least_squares

# --------- MDH 参数表 ----------
MDH_params = [
    [np.pi,         0.105,       0.0,         0.0],        # Joint 1
    [0.0,           0.0,         0.0,         np.pi/2],    # Joint 2
    [-2.83491633,   0.0,         0.18,        -np.pi],     # Joint 3
    [-2.83491266,   0.0,         0.18880943,  np.pi],      # Joint 4
    [-np.pi/2,      -0.00000016, -0.08,       np.pi/2],    # Joint 5
    [0.0,           0.036,       0.0,         np.pi/2],    # Joint 6
]

# --------- FK 函数 ----------
def mdh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -d*sa],
        [st*sa, ct*sa, ca, d*ca],
        [0, 0, 0, 1]
    ])

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

# --------- 初始猜测 ----------
q0 = np.zeros(6)

# --------- 调用优化器 ----------
res = least_squares(error_func, q0, method="trf")

print("求解到的关节角 qpos =")
print(res.x)
print("验证末端位置 =", forward_kinematics(res.x))

