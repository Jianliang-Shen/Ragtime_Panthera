#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>

void get_current_pose()
{
    // 初始化MoveGroupInterface
    // 假设机械臂的规划组名称为 "arm"
    moveit::planning_interface::MoveGroupInterface move_group("arm");

    // 获取当前末端执行器的位姿
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

    // 打印位姿信息
    ROS_INFO("Current End-Effector Pose:");
    ROS_INFO("Position: [x: %f, y: %f, z: %f]",
             current_pose.pose.position.x,
             current_pose.pose.position.y,
             current_pose.pose.position.z);
    ROS_INFO("Orientation: [x: %f, y: %f, z: %f, w: %f]",
             current_pose.pose.orientation.x,
             current_pose.pose.orientation.y,
             current_pose.pose.orientation.z,
             current_pose.pose.orientation.w);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "read_current_pose");
    ros::NodeHandle node_handle;

    // 启动异步spinner，用于处理MoveIt!的ROS回调
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 设置循环频率（例如 1Hz）
    ros::Rate loop_rate(1);  // 1Hz = 每秒1次

    ROS_INFO("Starting to publish current end-effector pose...");
    
     while (ros::ok())  // 只要ROS正常运行，就持续循环
    {
        get_current_pose();  // 调用获取位姿的函数
        loop_rate.sleep();   // 控制循环频率
    }

    // 关闭ROS节点
    ros::shutdown();
    return 0;
}

// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <termios.h>
// #include <unistd.h>
// #include <stdio.h>
// #include <atomic>
// #include <thread>

// // 全局变量：目标角度和运动标志
// std::atomic<double> target_angle(0.0);
// std::atomic<bool> running(true);

// // 获取键盘输入
// char getch()
// {
//     char buf = 0;
//     struct termios old = {0};
//     if (tcgetattr(0, &old) < 0)
//         perror("tcsetattr()");
//     old.c_lflag &= ~ICANON;
//     old.c_lflag &= ~ECHO;
//     old.c_cc[VMIN] = 1;
//     old.c_cc[VTIME] = 0;
//     if (tcsetattr(0, TCSANOW, &old) < 0)
//         perror("tcsetattr ICANON");
//     if (read(0, &buf, 1) < 0)
//         perror("read()");
//     old.c_lflag |= ICANON;
//     old.c_lflag |= ECHO;
//     if (tcsetattr(0, TCSADRAIN, &old) < 0)
//         perror("tcsetattr ~ICANON");
//     return buf;
// }

// // 键盘监听线程
// void keyboardListener()
// {
//     while (running)
//     {
//         char key = getch();
//         switch (key)
//         {
//             case 'w': // 按下w键，增加目标角度
//                 target_angle.store(target_angle.load() + 0.1);
//                 break;
//             case 's': // 按下s键，减少目标角度
//                 target_angle.store(target_angle.load() - 0.1);
//                 break;
//             case 'x': // 按下x键，退出程序
//                 running = false;
//                 break;
//             default:
//                 ROS_WARN("Invalid key pressed. Use 'w' to increase, 's' to decrease, 'x' to exit.");
//                 break;
//         }
//     }
// }

// int main(int argc, char** argv)
// {
//     // 初始化ROS节点
//     ros::init(argc, argv, "continuous_joint_control");
//     ros::NodeHandle node_handle;

//     // 启动异步spinner，用于处理MoveIt!的ROS回调
//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     // 初始化MoveGroupInterface
//     moveit::planning_interface::MoveGroupInterface move_group("arm");

//     // 设置第五轴关节的名称（根据你的URDF文件修改）
//     const std::string joint_name = "joint5";

//     // 启动键盘监听线程
//     std::thread keyboard_thread(keyboardListener);
//     double  last_target = 0.0;
//     // 控制循环
//     while (running)
//     {
//         // 获取当前关节状态
//         std::vector<double> current_joint_values = move_group.getCurrentJointValues();

//         // 获取第五轴关节的当前角度
//         double current_angle = current_joint_values[4]; // 假设第五轴是索引4
        
//         // 检查角度是否超出范围
//         if (target_angle > 1.5708) // 90度 = 1.5708弧度
//         {
//             ROS_ERROR("Joint 5 angle exceeds 90 degrees! Current angle: %f", target_angle.load());
//             target_angle = 1.5708; // 限制角度为90度
//         }
//         else if (target_angle < -1.5708) // -90度 = -1.5708弧度
//         {
//             ROS_ERROR("Joint 5 angle is below -90 degrees! Current angle: %f", target_angle.load());
//             target_angle = -1.5708; // 限制角度为-90度
//         }

//         // 更新目标角度
//         current_joint_values[4] = target_angle;

//         // 设置目标关节角度
//         move_group.setJointValueTarget(current_joint_values);

//         // 规划并执行运动
//         moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//         bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//         if (success && (last_target!=target_angle))
//         {
//             move_group.execute(my_plan);
//             ROS_INFO("Joint 5 moved to: %f radians", target_angle.load());
//         }
//         else
//         {
//             ROS_WARN("Failed to plan motion");
//         }
//         last_target = target_angle;
//         // 控制频率
//         ros::Duration(0.1).sleep(); // 100ms
//     }

//     // 等待键盘线程结束
//     keyboard_thread.join();

//     return 0;
// }