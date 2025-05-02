#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "damiao.h"
#include "unistd.h"
#include <cmath>
#include <algorithm> // for std::max and std::min
#include <iostream>  // for std::cout

damiao::Motor M1(damiao::DM4310,0x01, 0x11);
damiao::Motor M2(damiao::DM4340,0x02, 0x12);
damiao::Motor M3(damiao::DM4340,0x03, 0x13);
damiao::Motor M4(damiao::DM4340,0x04, 0x14);
damiao::Motor M5(damiao::DM4310,0x05, 0x15);
damiao::Motor M6(damiao::DM4310,0x06, 0x16);
damiao::Motor M7(damiao::DMH3510,0x07, 0x17);
std::shared_ptr<SerialPort> serial1;
std::shared_ptr<SerialPort> serial2;
damiao::Motor_Control dm1(serial1);
damiao::Motor_Control dm2(serial2);

int main(int argc, char **argv){
    ros::init(argc, argv, "dm_joint_states_pub");
    ros::NodeHandle nh;
    ROS_INFO("joint_states_pub node is Ready!");
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    sensor_msgs::JointState joint_state;
    serial1 = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    serial2 = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    dm1 = damiao::Motor_Control(serial1);
    dm2 = damiao::Motor_Control(serial2);
    dm1.addMotor(&M1);
    dm1.addMotor(&M2);
    dm1.addMotor(&M3);
    dm2.addMotor(&M4);
    dm2.addMotor(&M5);
    dm2.addMotor(&M5);
    dm2.addMotor(&M6);
    dm2.addMotor(&M7);
    joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6","gripper_1_joint"};
    joint_state.position = {0, 0, 0, 0, 0, 0, 0};
    joint_state.velocity = {0, 0, 0, 0, 0, 0};
    ros::Rate rate(500);
    while(ros::ok()){
        dm1.refresh_motor_status(M1);
        dm1.refresh_motor_status(M2);
        dm1.refresh_motor_status(M3);
        dm2.refresh_motor_status(M4);
        dm2.refresh_motor_status(M5);
        dm2.refresh_motor_status(M6);
        dm2.refresh_motor_status(M7);
        joint_state.header.stamp = ros::Time::now();
        joint_state.position[0] = M1.Get_Position();
        joint_state.position[1] = M2.Get_Position();
        joint_state.position[2] = M3.Get_Position();
        joint_state.position[3] = M4.Get_Position();
        joint_state.position[4] = M5.Get_Position();
        joint_state.position[5] = M6.Get_Position();
        joint_state.position[6] = M7.Get_Position();
        
        // joint_state.velocity[0] = M1.Get_Velocity();
        // joint_state.velocity[1] = M2.Get_Velocity();
        // joint_state.velocity[2] = M3.Get_Velocity();
        // joint_state.velocity[3] = M4.Get_Velocity();
        // joint_state.velocity[4] = M5.Get_Velocity();
        // joint_state.velocity[5] = M6.Get_Velocity();
        joint_state_pub.publish(joint_state);
        ROS_INFO("Real:[%f] [%f] [%f] [%f] [%f] [%f] ",joint_state.position[0],joint_state.position[1],joint_state.position[2],joint_state.position[3],joint_state.position[4],joint_state.position[5]);
        // ROS_INFO("Real:[%f] [%f] [%f] [%f] [%f] [%f] ",joint_state.velocity[0],joint_state.velocity[1],joint_state.velocity[2],joint_state.velocity[3],joint_state.velocity[4],joint_state.velocity[5]);

        rate.sleep();
    }
    return 0;
  }
  