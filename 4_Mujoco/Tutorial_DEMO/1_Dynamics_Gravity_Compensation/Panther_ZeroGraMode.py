import pinocchio as pin
import serial
import time
import threading

from Python_Panthera.damiao_controller.DM_CAN import *

# 初始化电机和串口
Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
Motor2 = Motor(DM_Motor_Type.DM4340, 0x02, 0x12)
Motor3 = Motor(DM_Motor_Type.DM4340, 0x03, 0x13)
Motor4 = Motor(DM_Motor_Type.DM4340, 0x04, 0x14)
Motor5 = Motor(DM_Motor_Type.DM4310, 0x05, 0x15)
Motor6 = Motor(DM_Motor_Type.DM4310, 0x06, 0x16)

serial_device1 = serial.Serial('COM13', 921600, timeout=0.5)
# serial_device2 = serial.Serial('COM15', 921600, timeout=0.5)
dm1 = MotorControl(serial_device1)
# dm2 = MotorControl(serial_device2)

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
dm1.enable(Motor1)
time.sleep(0.1)
dm1.enable(Motor2)
time.sleep(0.1)
dm1.enable(Motor3)
time.sleep(0.1)
dm1.enable(Motor4)
time.sleep(0.1)
dm1.enable(Motor5)
time.sleep(0.1)
dm1.enable(Motor6)

# 加载URDF模型（替换为你的实际路径）
urdf_path = "panther_description.urdf"
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# 设置重力（z轴向下）
model.gravity.linear = np.array([0, 0, -9.81])

# 共享变量
q = np.zeros(8)  # 关节位置
vel = np.zeros(8)  # 关节速度
f = np.zeros(6)  # 摩擦力补偿
tau = np.zeros(6)  # 最终力矩
data_lock = threading.Lock()

# 重力补偿计算函数
def compute_gravity_compensation(q):
    pin.computeGeneralizedGravity(model, data, q)
    return data.g

# 摩擦力补偿（与C++代码保持一致）
def compute_friction_compensation(vel):
    f = np.zeros(6)
    if vel[0] < -0.006: f[0] = 0.06 * vel[0]
    if vel[0] > 0.006: f[0] = 0.06 * vel[0]
    
    if vel[1] < -0.006: f[1] = -0.1 + 0.05 * vel[1]
    if vel[1] > 0.006: f[1] = 0.1 + 0.05 * vel[1]
    
    if vel[2] < -0.006: f[2] = -0.15 + 0.05 * vel[2]
    if vel[2] > 0.006: f[2] = 0.15 + 0.05 * vel[2]
    
    if vel[3] < -0.006: f[3] = -0.30 + 0.2 * vel[3]
    if vel[3] > 0.006: f[3] = 0.30 + 0.2 * vel[3]
    
    if vel[4] < -0.01: f[4] = 0.03 - 0.05 * vel[4]
    if vel[4] > 0.01: f[4] = -0.03 - 0.05 * vel[4]
    
    return f

# 500Hz重力补偿线程
def gravity_compensation_loop():
    control_rate = 400  # Hz
    control_period = 1.0 / control_rate
    loop_count = 0
    start_time_total = time.time()
    
    while True:
        start_time = time.time()
        
        # 刷新电机状态
        dm1.refresh_motor_status(Motor1)
        dm1.refresh_motor_status(Motor2)
        dm1.refresh_motor_status(Motor3)
        dm1.refresh_motor_status(Motor4)
        dm1.refresh_motor_status(Motor5)
        dm1.refresh_motor_status(Motor6)
        
        # 更新共享变量
        with data_lock:
            q[0] = Motor1.getPosition()
            q[1] = Motor2.getPosition()
            q[2] = Motor3.getPosition()
            q[3] = Motor4.getPosition()
            q[4] = Motor5.getPosition()
            q[5] = Motor6.getPosition()
            q[6] = 0
            q[7] = 0
            
            # vel[0] = Motor1.getVelocity()
            # vel[1] = Motor2.getVelocity()
            # vel[2] = Motor3.getVelocity()
            # vel[3] = Motor4.getVelocity()
            # vel[4] = Motor5.getVelocity()
            # vel[5] = Motor6.getVelocity()
            vel[6] = 0
            vel[7] = 0
        
        # 计算补偿力矩
        tau_gravity = compute_gravity_compensation(q)
        # f = compute_friction_compensation(vel)
        # tau = tau_gravity + f
        tau = tau_gravity
        print(tau)

        # 发送力矩指令
        dm1.controlMIT(Motor1, 0.0, 0.0, 0.0, 0.0, tau[0])
        dm1.controlMIT(Motor2, 0.0, 0.0, 0.0, 0.0, tau[1])
        dm1.controlMIT(Motor3, 0.0, 0.0, 0.0, 0.0, tau[2])
        dm1.controlMIT(Motor4, 0.0, 0.0, 0.0, 0.0, tau[3])
        dm1.controlMIT(Motor5, 0.0, 0.0, 0.0, 0.0, tau[4])
        dm1.controlMIT(Motor6, 0.0, 0.0, 0.0, 0.0, tau[5])

        # stop_time = time.time()
        # time_diff = stop_time - start_time
        # freq = 1 / (time_diff + 1e-9)  # Add small epsilon to avoid division by zero
        # print(f"Frequency: {freq}")
            
        # 精确睡眠控制
        elapsed = time.time() - start_time
        # print(f"Elapsed time: {1.0/(elapsed+ 1e-9)}")
        sleep_time = max(0, control_period - elapsed - 0.0001)  # 补偿微小延迟
        time.sleep(sleep_time)
        # print(f"Sleep time: {sleep_time}")
        

# 启动控制线程
gc_thread = threading.Thread(target=gravity_compensation_loop, daemon=True)
gc_thread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    # 禁用所有电机
    dm1.disable(Motor1)
    dm1.disable(Motor2)
    dm1.disable(Motor3)
    dm1.disable(Motor4)
    dm1.disable(Motor5)
    dm1.disable(Motor6)
    print("程序终止，所有电机已禁用")