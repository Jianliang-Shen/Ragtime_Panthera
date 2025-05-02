#include <ros/ros.h>
#include <std_msgs/String.h>
#include "damiao.h"
#include "unistd.h"
#include <cmath>
#include <algorithm> // for std::max and std::min
#include <iostream>  // for std::cout

damiao::Motor M7(damiao::DMH3510,0x07, 0x17);
std::shared_ptr<SerialPort> serial;
damiao::Motor_Control dm(serial);

class GripperController
{
public:
  GripperController()
  {
    // 初始化订阅者，话题名为"gripper_control"
    sub_ = nh_.subscribe("gripper_control", 10, &GripperController::callback, this);
    ROS_INFO("Gripper controller node ready. Waiting for commands...");
  }

  void callback(const std_msgs::String::ConstPtr& msg)  // 修改回调参数类型
  {
    const std::string& command = msg->data;
    
    // 添加字符串指令判断
    if (command == "open") {
      ROS_INFO("Received OPEN command");
      openGripper();
    } 
    else if (command == "close") {
      ROS_INFO("Received CLOSE command");
      closeGripper();
    }
    else {
      ROS_WARN("Received unknown command: %s", command.c_str());
    }
  }

  //-----------------------------
  // 电机控制接口（需要您实现具体硬件逻辑）
  //-----------------------------
  virtual void openGripper()
  {
    dm.control_pos_force(M7,2.0f,500,500);
    ROS_INFO("Gripper open success!!");
  }

  virtual void closeGripper()
  {
    dm.control_pos_force(M7,-2.0f,500,500);
    ROS_INFO("Gripper close success!!");
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_controller");
  serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
  dm = damiao::Motor_Control(serial);
  GripperController controller;

  dm.addMotor(&M7);
  dm.enable(M7);

  ros::spin();

  dm.disable(M7);
  return 0;
}