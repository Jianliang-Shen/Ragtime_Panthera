#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Float64.h>

# include <actionlib/server/simple_action_server.h>
# include <control_msgs/FollowJointTrajectoryAction.h>
# include <std_msgs/Float32MultiArray.h>
# include <iostream>
# include <moveit_msgs/RobotTrajectory.h>

#include <vector>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/duration.h>

#include "std_msgs/String.h"
#include "damiao.h"
#include "unistd.h"
#include <cmath>
#include <algorithm> // for std::max and std::min
#include <iostream>  // for std::cout

damiao::Motor M1(damiao::DM4340,0x01, 0x11);
damiao::Motor M2(damiao::DM4340,0x02, 0x12);
damiao::Motor M3(damiao::DM4340,0x03, 0x13);
damiao::Motor M4(damiao::DM4310,0x04, 0x14);
damiao::Motor M5(damiao::DM4310,0x05, 0x15);
damiao::Motor M6(damiao::DM4310,0x06, 0x16);
std::shared_ptr<SerialPort> serial;
damiao::Motor_Control dm(serial);

class MyArmHW : public hardware_interface::RobotHW
{
public:
    MyArmHW()
    {
        // 初始化关节状态接口
        hardware_interface::JointStateHandle state_handle_1("joint1", &joint_position[0], &joint_velocity[0], &joint_effort[0]);
        jnt_state_interface.registerHandle(state_handle_1);

        hardware_interface::JointStateHandle state_handle_2("joint2", &joint_position[1], &joint_velocity[1], &joint_effort[1]);
        jnt_state_interface.registerHandle(state_handle_2);

        hardware_interface::JointStateHandle state_handle_3("joint3", &joint_position[2], &joint_velocity[2], &joint_effort[2]);
        jnt_state_interface.registerHandle(state_handle_3);

        hardware_interface::JointStateHandle state_handle_4("joint4", &joint_position[3], &joint_velocity[3], &joint_effort[3]);
        jnt_state_interface.registerHandle(state_handle_4);

        hardware_interface::JointStateHandle state_handle_5("joint5", &joint_position[4], &joint_velocity[4], &joint_effort[4]);
        jnt_state_interface.registerHandle(state_handle_5);
        
        hardware_interface::JointStateHandle state_handle_6("joint6", &joint_position[5], &joint_velocity[5], &joint_effort[5]);
        jnt_state_interface.registerHandle(state_handle_6);

        registerInterface(&jnt_state_interface);

        // 初始化关节命令接口
        hardware_interface::JointHandle  pos_handle_1(jnt_state_interface.getHandle("joint1"), &joint_position_command[0]);
        jnt_pos_interface.registerHandle(pos_handle_1);

        hardware_interface::JointHandle  pos_handle_2(jnt_state_interface.getHandle("joint2"), &joint_position_command[1]);
        jnt_pos_interface.registerHandle(pos_handle_2);

        hardware_interface::JointHandle  pos_handle_3(jnt_state_interface.getHandle("joint3"), &joint_position_command[2]);
        jnt_pos_interface.registerHandle(pos_handle_3);

        hardware_interface::JointHandle  pos_handle_4(jnt_state_interface.getHandle("joint4"), &joint_position_command[3]);
        jnt_pos_interface.registerHandle(pos_handle_4);

        hardware_interface::JointHandle  pos_handle_5(jnt_state_interface.getHandle("joint5"), &joint_position_command[4]);
        jnt_pos_interface.registerHandle(pos_handle_5);

        hardware_interface::JointHandle  pos_handle_6(jnt_state_interface.getHandle("joint6"), &joint_position_command[5]);
        jnt_pos_interface.registerHandle(pos_handle_6);

        registerInterface(&jnt_pos_interface);

        hardware_interface::JointHandle vel_handle_1(jnt_state_interface.getHandle("joint1"), &joint_velocity_command[0]);
        jnt_vel_interface.registerHandle(vel_handle_1);

        hardware_interface::JointHandle vel_handle_2(jnt_state_interface.getHandle("joint2"), &joint_velocity_command[1]);
        jnt_vel_interface.registerHandle(vel_handle_2);

        hardware_interface::JointHandle vel_handle_3(jnt_state_interface.getHandle("joint3"), &joint_velocity_command[2]);
        jnt_vel_interface.registerHandle(vel_handle_3);

        hardware_interface::JointHandle vel_handle_4(jnt_state_interface.getHandle("joint4"), &joint_velocity_command[3]);
        jnt_vel_interface.registerHandle(vel_handle_4);

        hardware_interface::JointHandle vel_handle_5(jnt_state_interface.getHandle("joint5"), &joint_velocity_command[4]);
        jnt_vel_interface.registerHandle(vel_handle_5);

        hardware_interface::JointHandle vel_handle_6(jnt_state_interface.getHandle("joint6"), &joint_velocity_command[5]);
        jnt_vel_interface.registerHandle(vel_handle_6);

        registerInterface(&jnt_vel_interface);
    }

    void read()
    {
        // 读取电机状态
        dm.refresh_motor_status(M1);
        dm.refresh_motor_status(M2);
        dm.refresh_motor_status(M3);
        dm.refresh_motor_status(M4);
        dm.refresh_motor_status(M5);
        dm.refresh_motor_status(M6);

        joint_position[0] = M1.Get_Position();
        joint_position[1] = M2.Get_Position();
        joint_position[2] = M3.Get_Position();
        joint_position[3] = M4.Get_Position();
        joint_position[4] = M5.Get_Position();
        joint_position[5] = M6.Get_Position();

        joint_velocity[0] = M1.Get_Velocity();
        joint_velocity[1] = M2.Get_Velocity();
        joint_velocity[2] = M3.Get_Velocity();
        joint_velocity[3] = M4.Get_Velocity();
        joint_velocity[4] = M5.Get_Velocity();
        joint_velocity[5] = M6.Get_Velocity();

    }

    void write()
    {
        // 写入电机命令
        dm.control_pos_vel(M1, joint_position_command[0], joint_velocity_command[0]);  // 假设速度控制为0
        dm.control_pos_vel(M2, joint_position_command[1], joint_velocity_command[1]);
        dm.control_pos_vel(M3, joint_position_command[2], joint_velocity_command[2]);
        dm.control_pos_vel(M4, joint_position_command[3], joint_velocity_command[3]);
        dm.control_pos_vel(M5, joint_position_command[4], joint_velocity_command[4]);
        dm.control_pos_vel(M6, joint_position_command[5], joint_velocity_command[5]);
    }

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;

    double joint_position[6];
    double joint_velocity[6];
    double joint_effort[6];
    double joint_position_command[6];
    double joint_velocity_command[6];
    //double joint_effort_command[6];
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_arm_hw");
    ros::NodeHandle nh;

    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    dm = damiao::Motor_Control(serial);

    dm.addMotor(&M1);
    dm.addMotor(&M2);
    dm.addMotor(&M3);
    dm.addMotor(&M4);
    dm.addMotor(&M5);
    dm.addMotor(&M6);

    // dm.enable(M1);
    // dm.enable(M2);
    // dm.enable(M3);
    // dm.enable(M4);
    // dm.enable(M5);
    dm.enable(M6);
    // 创建机器人实例，以便可以获取可用资源
    MyArmHW my_arm_hw;
    // 创建一个控制器管理器实例，并将机器人对象指针作为参数传入，以便其可以处理机器人资源
    controller_manager::ControllerManager cm(&my_arm_hw, nh);

    // 设置一个线程用于接收ROS服务
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // 设置控制循环
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0); // 10 Hz rate

    ROS_INFO("Hardware interface OK!!");
    while (ros::ok())
    {
        // 计算控制周期
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;

        prev_time = time;
        
        my_arm_hw.read();
        cm.update(ros::Time::now(), rate.expectedCycleTime());
        my_arm_hw.write();
        rate.sleep();
    }
    sleep(0.1);
    // dm.disable(M1);
    // dm.disable(M2);
    // dm.disable(M3);
    // dm.disable(M4);
    // dm.disable(M5);
    dm.disable(M6);
    sleep(0.1);
    return 0;
}