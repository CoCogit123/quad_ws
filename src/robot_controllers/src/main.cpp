// **************************
// 自定义头文件 
// **************************
#include "Dynamics.h" //有Pinocchio必须放在最前面
#include "Utils.h"
#include "Gait.h"
#include "Swing.h"
#include "Debug.h"
//自定义消息包
#include <custom_msgs/Motor_state.h>
#include <custom_msgs/Motor_control.h>
#include <custom_msgs/Sim_info.h>
#include <custom_msgs/Joy_control.h>
// **************************
// 其他头文件 
// **************************
#include <ros/ros.h>
#include <thread>
#include <string>
#include <iostream>
#include <iomanip> // 用于控制输出格式
#include <ros/package.h> // 用于获取功能包路径
#include <sensor_msgs/Imu.h>//imu消息包

using namespace controllers;//自定义工作空间

int main(int argc, char** argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh("~");
    ROS_INFO("Starting Main ...");

    // **************************
    // 全局结构体和类 
    // **************************
    Robot_info robot_info;//机器人信息
    Gait_info gait_info;//步态信息     
    Swing_info swing_info;//摆动相信息

    //类
    std::string urdf_pkg_dir = ros::package::getPath("robot_description");//urdf包路径
    std::string urdf_path = urdf_pkg_dir +  "/robot/urdf/robot.urdf";//urdf路径
    ROS_INFO("succes urdf_path: %s", urdf_path.c_str());
    Dynamics dynamics_solver(urdf_path);
    ROS_INFO("Dynamics Solver Initialized Successfully.");
    Gait Gait_solver;
    ROS_INFO("Gait Solver Initialized Successfully.");
    Swing Swing_solver;
    ROS_INFO("Swing Solver Initialized Successfully.");
    // **************************
    // 订阅节点 设置回调函数
    // **************************
        // 使用 Lambda 捕获局部变量的引用 [&]
        //必须指定消息类型了
    ros::Subscriber Motor_state_sub = nh.subscribe<custom_msgs::Motor_state>(
        "/Motor_state", 
        10, 
        [&robot_info](const custom_msgs::Motor_state::ConstPtr& msg) {
            // 在这里可以直接访问并修改局部变量
            robot_info.Pos_motor = Eigen::Map<const Vector12d>(msg->pos.data());
            robot_info.Vel_motor = Eigen::Map<const Vector12d>(msg->vel.data());
        }
    );
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
        "/imu", 10, 
        [&robot_info](const sensor_msgs::Imu::ConstPtr& msg) {
            //一阶滤波过滤数据
            // 定义滤波系数 (alpha 越小越平滑，但延迟越大)
            const double alpha = 0.2; 
            //获取当前帧原始数据
            Eigen::Vector3d raw_acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            Eigen::Vector3d raw_omega(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            //使用静态变量存储上一时刻的状态（初次运行时初始化为当前值）
            static Eigen::Vector3d last_acc = raw_acc;
            static Eigen::Vector3d last_omega = raw_omega;
            //执行一阶低通滤波公式: y(k) = alpha * x(k) + (1 - alpha) * y(k-1)
            robot_info.body_Acc = alpha * raw_acc + (1.0 - alpha) * last_acc;
            robot_info.body_Omega = alpha * raw_omega + (1.0 - alpha) * last_omega;
            //更新缓存，供下一帧使用
            last_acc = robot_info.body_Acc;
            last_omega = robot_info.body_Omega;

            //处理四元数 (注意：ROS是x,y,z,w; Eigen构造函数是w,x,y,z)
            robot_info.quat_base = Eigen::Quaterniond(msg->orientation.w,msg->orientation.x,msg->orientation.y, msg->orientation.z);
            //转换旋转矩阵
            robot_info.body_Rot_world = robot_info.quat_base.toRotationMatrix();
            // 转换欧拉角 (Roll, Pitch, Yaw)
            double q0 = robot_info.quat_base.w(); double q1 = robot_info.quat_base.x(); double q2 = robot_info.quat_base.y(); double q3 = robot_info.quat_base.z();
            double roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
            double pitch = asin(2*(q0*q2 - q3*q1));
            double yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
            robot_info.euler << roll, pitch, yaw; // 重新排布为 R,P,Y

            robot_info.world_Acc = robot_info.body_Rot_world*robot_info.body_Acc - Vector3d(0, 0, 9.81);;
        }
    );
    //仿真作弊用
    ros::Subscriber Sim_info_sub = nh.subscribe<custom_msgs::Sim_info>(
        "/Sim_info", 
        10, 
        [&robot_info](const custom_msgs::Sim_info::ConstPtr& msg) {
            // 在这里可以直接访问并修改局部变量
            robot_info.world_Pos_com = Vector3d(msg->sim_pos.x, msg->sim_pos.y, msg->sim_pos.z);
            robot_info.body_Vel_com = Vector3d(msg->sim_twist.linear.x,msg->sim_twist.linear.y,msg->sim_twist.linear.z);
            robot_info.world_Vel_com = robot_info.body_Rot_world*robot_info.body_Vel_com;
        }
    );
    ros::Subscriber Joy_sub = nh.subscribe<custom_msgs::Joy_control>(
        "/joy_control", 
        10, 
        [&robot_info, &gait_info](const custom_msgs::Joy_control::ConstPtr& msg) {
            if(msg->run_flag == true)
            {
                // 1. 存储线速度（机体系）
                robot_info.body_Vel_des = Vector3d(msg->com_vel.x, msg->com_vel.y, 0.0);
                
                // 2. 存储角速度（机体系）- 假设 msg->yaw_vel 对应绕 Z 轴角速度
                robot_info.body_omega_des = Vector3d(0.0, 0.0, msg->yaw_vel);

                // 3. 根据 Z 方向速度更新期望高度 z_des (数值积分)
                // 注意：这里需要乘以 dt，或者给一个步进增量。假设控制周期约为 10ms
                double dt = 0.01; 
                robot_info.z_des += msg->com_vel.z * dt;
                // 4. 设置高度限幅 (例如 0.1m 到 0.5m)
                robot_info.z_des = std::max(0.1, std::min(0.5, robot_info.z_des));

                robot_info.euler_des[2] = robot_info.euler[2] + msg->yaw_vel*dt;

                // 5. 读取 mode 并转换为步态枚举存储
                // 假设你的枚举强制转换是安全的
                gait_info.Gait_des = static_cast<Gait_type>(msg->mode);
                if(gait_info.Gait_des!=gait_info.Gait_mode)
                {
                    gait_info.Gait_flag=1;
                }
            }else 
            {
                robot_info.body_Vel_des.setZero();
                robot_info.body_omega_des.setZero();
            }
            robot_info.world_Vel_des = robot_info.body_Rot_world*robot_info.body_Vel_des;
            robot_info.world_omega_des = robot_info.body_omega_des;
        }
    );

    // =========================================================
    // 线程 1  (500Hz) 
    // =========================================================
    std::thread thread_high([&]() {
        double target_freq = 500.0;//目标hz
        double expected_cycle_time = 1.0 / target_freq; // 0.002s 目标delta_t
        ros::Rate rate(target_freq);
        // 初始化时间记录 ros::WallTime不会被暂停
        ros::WallTime last_start_time = ros::WallTime::now();
        ros::WallTime current_start_time = ros::WallTime::now();
        //运行时间
        ros::WallTime thread_start_time = ros::WallTime::now();
        double thread_runtime=0.0;
        //delta_t
        double thread_delta_t_;
        while (ros::ok()) {
            // 记录循环开始时间
            current_start_time = ros::WallTime::now();
            thread_delta_t_ = (current_start_time - last_start_time).toSec();
            last_start_time = current_start_time;
            thread_runtime = (current_start_time-thread_start_time).toSec();
            // ---------------------------------------------
            // 核心控制代码
            // ---------------------------------------------
            //Pinocchio
            dynamics_solver.update(robot_info); 
            //Gait
            Gait_solver.update(gait_info,thread_runtime);
            //Swing
            Swing_solver.update(swing_info,robot_info,gait_info);
            // 打印调试信息 (每1秒打印一次，避免刷屏)
            ROS_INFO_STREAM_THROTTLE(1.0, 
                "\n[500Hz Thread]"
                << "\n  Rate     : " << std::fixed << std::setprecision(2) << (1.0 / thread_delta_t_) << " Hz"
                << "\n  Delta T  : " << std::setprecision(6) << thread_delta_t_ << " s"
            );
            // Debug_robot_info(robot_info,1);
            // Debug_gait_info(gait_info,1);
            Debug_swing_info(swing_info,10);
            // 休眠对齐频率
            rate.sleep();
        }
    });

    // =========================================================
    // 线程 2 (100Hz) 
    // =========================================================
    std::thread thread_low([&]() {
        double target_freq = 100.0;//目标hz
        double expected_cycle_time = 1.0 / target_freq; // 0.01s 目标delta_t
        ros::Rate rate(target_freq);
        // 初始化时间记录 ros::WallTime不会被暂停
        ros::WallTime last_start_time = ros::WallTime::now();
        ros::WallTime current_start_time = ros::WallTime::now();

        double thread_delta_t_;//delta_t
        while (ros::ok()) {
            // 记录循环开始时间
            current_start_time = ros::WallTime::now();
            thread_delta_t_ = (current_start_time - last_start_time).toSec();
            last_start_time = current_start_time;

            // ---------------------------------------------
            // 核心控制代码
            // ---------------------------------------------
             
            // ...

            // 打印调试信息 (每1秒打印一次，避免刷屏)
            // ROS_INFO_STREAM_THROTTLE(1.0, 
            //     "\n[100Hz Thread]"
            //     << "\n  Rate     : " << std::fixed << std::setprecision(2) << (1.0 / thread_delta_t_) << " Hz"
            //     << "\n  Delta T  : " << std::setprecision(6) << thread_delta_t_ << " s"
            // );

            // 休眠对齐频率
            rate.sleep();
        }
    });

    // =========================================================
    // 主线程逻辑
    // =========================================================
    
    // 如果你有 Subscriber 回调需要处理，使用 ros::spin()
    // 如果没有回调，ros::spin() 只是作为一个阻塞器防止 main 函数退出
    ros::spin();

    // 当 Ctrl+C 按下，ros::ok() 变为 false，上面的 while 循环终止
    // 等待线程安全退出
    if (thread_high.joinable()) thread_high.join();
    if (thread_low.joinable()) thread_low.join();

    ROS_INFO("Main Node Exited Cleanly.");
    return 0;
}