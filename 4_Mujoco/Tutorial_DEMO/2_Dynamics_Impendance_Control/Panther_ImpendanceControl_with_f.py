import pinocchio as pin
import serial
import time
import threading

from Python_Panthera.damiao_controller.DM_CAN import *

# ---------- 电机初始化 ----------
# 初始化电机和串口
Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
Motor2 = Motor(DM_Motor_Type.DM4340, 0x02, 0x12)
Motor3 = Motor(DM_Motor_Type.DM4340, 0x03, 0x13)
Motor4 = Motor(DM_Motor_Type.DM4340, 0x04, 0x14)
Motor5 = Motor(DM_Motor_Type.DM4310, 0x05, 0x15)
Motor6 = Motor(DM_Motor_Type.DM4310, 0x06, 0x16)

serial_device1 = serial.Serial('COM13', 921600, timeout=0.5)
dm1 = MotorControl(serial_device1)

# 添加电机
dm1.addMotor(Motor1)
dm1.addMotor(Motor2)
dm1.addMotor(Motor3)
dm1.addMotor(Motor4)
dm1.addMotor(Motor5)
dm1.addMotor(Motor6)

# 设置控制模式
dm1.switchControlMode(Motor1, Control_Type.MIT)
dm1.switchControlMode(Motor2, Control_Type.MIT)
dm1.switchControlMode(Motor3, Control_Type.MIT)
dm1.switchControlMode(Motor4, Control_Type.MIT)
dm1.switchControlMode(Motor5, Control_Type.MIT)
dm1.switchControlMode(Motor6, Control_Type.MIT)

# 使能电机
dm1.enable(Motor1);time.sleep(0.1)
dm1.enable(Motor2);time.sleep(0.1)
dm1.enable(Motor3);time.sleep(0.1)
dm1.enable(Motor4);time.sleep(0.1)
dm1.enable(Motor5);time.sleep(0.1)
dm1.enable(Motor6);time.sleep(0.1)

# ---------- URDF 模型加载 ----------
# 加载URDF模型（替换为你的实际路径）
urdf_path = "../1_Dynamics_Gravity_Compensation/panther_description.urdf"
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()
model.gravity.linear = np.array([0, 0, -9.81])

# ---------- 阻抗控制参数 ----------
# KLM
K = np.array([70, 85, 125, 160, 130, 350])#刚度系数k
B = np.array([10, 10, 11, 10, 8, 8])#阻尼系数b
# MIT
# K = np.array([3.5, 6.5, 8.5, 2.5, 1.5, 0.80])#刚度系数k
# B = np.array([0.85, 1.05, 1.05, 0.35, 0.15, 0.13])#阻尼系数b
# 共享变量
q_des = np.array([0.0, 1.0, 1.0, 0.0, 0.0, 0.0])  # 期望目标位置
v_des = np.zeros(6)

# ---------- 阻抗控制线程 ----------
# 共享变量
q = np.zeros(8)  # 关节位置
vel = np.zeros(8)  # 关节速度
f = np.zeros(6)  # 摩擦力补偿
tau = np.zeros(6)  # 最终力矩
data_lock = threading.Lock()

def impedance_control_loop():
    control_rate = 400  # Hz
    control_period = 1.0 / control_rate

    while True:
        start_time = time.time()

        # 刷新电机状态
        dm1.refresh_motor_status(Motor1)
        dm1.refresh_motor_status(Motor2)
        dm1.refresh_motor_status(Motor3)
        dm1.refresh_motor_status(Motor4)
        dm1.refresh_motor_status(Motor5)
        dm1.refresh_motor_status(Motor6)

        # 读取关节位置和速度
        q[0] = Motor1.getPosition()
        q[1] = Motor2.getPosition()
        q[2] = Motor3.getPosition()
        q[3] = Motor4.getPosition()
        q[4] = Motor5.getPosition()
        q[5] = Motor6.getPosition()
        q[6] = 0
        q[7] = 0

        vel[0] = Motor1.getVelocity()
        vel[1] = Motor2.getVelocity()
        vel[2] = Motor3.getVelocity()
        vel[3] = Motor4.getVelocity()
        vel[4] = Motor5.getVelocity()
        vel[5] = Motor6.getVelocity()
        vel[6] = 0
        vel[7] = 0

        # 动力学项计算
        M_full = pin.crba(model, data, q)
        C_full = pin.computeCoriolisMatrix(model, data, q, vel)
        G_full = pin.computeGeneralizedGravity(model, data, q)#M是惯性矩阵，C是科里奥利矩阵，G是重力矩阵

        # 只提取前6个关节部分用于控制
        M = M_full[:6, :6]
        C = C_full[:6, :6]
        G = G_full[:6]

        if vel[0] < -0.006: f[0] = 0.03 * vel[0]
        if vel[0] > 0.006: f[0] = 0.03 * vel[0]

        if vel[1] < -0.006: f[1] = -0.1 + 0.05 * vel[1]
        if vel[1] > 0.006: f[1] = 0.1 + 0.05 * vel[1]
        
        if vel[2] < -0.006: f[2] = -0.15 + 0.05 * vel[2]
        if vel[2] > 0.006: f[2] = 0.15 + 0.05 * vel[2]
        
        if vel[3] < -0.006: f[3] = -0.30 + 0.2 * vel[3]
        if vel[3] > 0.006: f[3] = 0.30 + 0.2 * vel[3]
        
        if vel[4] < -0.01: f[4] = 0.03 - 0.05 * vel[4]
        if vel[4] > 0.01: f[4] = -0.03 - 0.05 * vel[4]

        # 计算阻抗控制输出力矩
        # KLM
        acc_des = K * (q_des - q[:6]) + B * (v_des - vel[:6])
        # tau = M @ acc_des + C @ vel[:6] + G[:6]
        # with f
        tau = M @ acc_des + C @ vel[:6] + G[:6] + f[:6]
        # tau = G[:6]
        # MIT
        # acc_des = K * (q_des - q[:6]) + B * (v_des - vel[:6])
        # tau = acc_des[:6] + C @ vel[:6] + G[:6] + f[:6]

        # print("tau_cal:", tau)

        # 力矩限幅（基于电机规格）
        tau_limit = np.array([5.5, 26.5, 26.5, 26.5, 5.5, 5.5])
        tau = np.clip(tau, -tau_limit, tau_limit)

        # print(M @ acc_des)
        print("tau_out:", tau)
        # print("q_cur (rad):", q[:6])
        # print("error (rad):", q_des - q[:6])
        # print(M)

        # 输出到电机
        dm1.controlMIT(Motor1, 0.0, 0.0, 0.0, 0.0, tau[0])
        dm1.controlMIT(Motor2, 0.0, 0.0, 0.0, 0.0, tau[1])
        dm1.controlMIT(Motor3, 0.0, 0.0, 0.0, 0.0, tau[2])
        dm1.controlMIT(Motor4, 0.0, 0.0, 0.0, 0.0, tau[3])
        dm1.controlMIT(Motor5, 0.0, 0.0, 0.0, 0.0, tau[4])
        dm1.controlMIT(Motor6, 0.0, 0.0, 0.0, 0.0, tau[5])

        # 控制频率
        elapsed = time.time() - start_time
        time.sleep(max(0, control_period - elapsed - 0.0001))

# ---------- 启动线程 ----------
ic_thread = threading.Thread(target=impedance_control_loop, daemon=True)
ic_thread.start()

# ---------- 主循环保持运行 ----------
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    dm1.disable(Motor1)
    dm1.disable(Motor2)
    dm1.disable(Motor3)
    dm1.disable(Motor4)
    dm1.disable(Motor5)
    dm1.disable(Motor6)
    print("程序终止，所有电机已禁用")
