import numpy as np

# MDH参数: theta0, d, a, alpha
MDH_params = [
    [np.pi,         0.105,          0.0,        0.0],       # Joint 1
    [0.0,           0.0,            0.0,        np.pi/2],   # Joint 2
    [-2.83491633,   0.0,            0.18,       -np.pi],    # Joint 3
    [-2.83491266,   -0.0,           0.18880943, np.pi],     # Joint 4
    [-np.pi/2,      -0.00000016,    -0.08,      np.pi/2],   # Joint 5
    [0.0,           0.036,          -0.0,       np.pi/2],   # Joint 6
]

# qpos 输入（弧度）
qpos = [0, np.pi/2, np.pi/2, 0, 0, 0]

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

T = np.eye(4)

print("Joint origins relative to base_link:")
for i, (params, q) in enumerate(zip(MDH_params, qpos), start=1):
    theta0, d, a, alpha = params
    theta = theta0 + q   # 加上关节变量
    A = mdh_transform(theta, d, a, alpha)
    T = T @ A
    pos = T[:3, 3]
    print(f"Joint {i}: x={pos[0]: .6f}, y={pos[1]: .6f}, z={pos[2]: .6f}")
