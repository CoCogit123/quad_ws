// **************************
// 自定义头文件 
// **************************
#include "Dynamics.h" //有Pinocchio必须放在最前面
#include "Utils.h"
#include "Gait.h"
#include "Swing.h"
#include "Debug.h"
#include "Manager.h"
#include "Estimate.h"
#include "Mpc.h"
#include "Wbc.h"
#include "Rviz_vision.h"
//自定义消息包
#include <custom_msgs/Motor_state.h>
#include <custom_msgs/Motor_control.h>
#include <custom_msgs/Sim_info.h>
#include <custom_msgs/Joy_control.h>
#include <custom_msgs/Debug_info.h>
// **************************
// 其他头文件 
// **************************
#include <ros/ros.h>
#include <thread>
#include <string>
#include <iostream>
#include <chrono> //系统时间
#include <iomanip> // 用于控制输出格式
#include <ros/package.h> // 用于获取功能包路径
#include <sensor_msgs/Imu.h>//imu消息包
#include <sensor_msgs/Joy.h> //joy消息包
#include <unistd.h> //CPU亲密性
#include <sys/syscall.h> //CPU亲密性
#include <sched.h>

using namespace controllers;//自定义工作空间
double get_loop_interval(void);
bool setThreadCpuAffinity(std::thread& thread, int cpu_core);//设置cpu的亲密性
bool setThreadPriority(std::thread& thread, int priority);//设置cpu的线程

#define using_time 0  //0：使用不停止的接近现实时间  1：使用mujoco时间
double time_mujoco;

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
    Wbc_info wbc_info;//wbc信息
    
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
    Manager Manager_solver;
    ROS_INFO("Manager Solver Initialized Successfully.");
    Estimate Estimate_solver;
    Mpc Mpc_solver;
    Wbc Wbc_solver(robot_info);
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
            static MovingWindowFilter acc_x_filter(5);
            static MovingWindowFilter acc_y_filter(5);
            static MovingWindowFilter acc_z_filter(5);
            static MovingWindowFilter omega_x_filter(5);
            static MovingWindowFilter omega_y_filter(5);
            static MovingWindowFilter omega_z_filter(5);

            robot_info.body_Acc = Eigen::Vector3d(
                acc_x_filter.CalculateAverage(msg->linear_acceleration.x),
                acc_y_filter.CalculateAverage(msg->linear_acceleration.y),
                acc_z_filter.CalculateAverage(msg->linear_acceleration.z)
                );
            robot_info.body_Omega = Eigen::Vector3d(
                omega_x_filter.CalculateAverage(msg->angular_velocity.x),
                omega_y_filter.CalculateAverage(msg->angular_velocity.y),
                omega_z_filter.CalculateAverage(msg->angular_velocity.z)
                );

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

            robot_info.world_Acc = robot_info.body_Rot_world*robot_info.body_Acc;
        }
    );
    //仿真作弊用
    ros::Subscriber Sim_info_sub = nh.subscribe<custom_msgs::Sim_info>(
        "/Sim_info", 
        10, 
        [&robot_info](const custom_msgs::Sim_info::ConstPtr& msg) {
            // 在这里可以直接访问并修改局部变量
            time_mujoco = msg->run_time;
            // robot_info.world_Pos_com = Vector3d(msg->sim_pos.x, msg->sim_pos.y, msg->sim_pos.z);
            // robot_info.world_Pos_com[2]-=0.0189;//减去足端半径 默认足端圆心为地面0处
            // robot_info.body_Vel_com = Vector3d(msg->sim_twist.linear.x,msg->sim_twist.linear.y,msg->sim_twist.linear.z);
            // robot_info.world_Vel_com = robot_info.body_Rot_world*robot_info.body_Vel_com;
        }
    );
    ros::Subscriber Keyboard_sub = nh.subscribe<custom_msgs::Joy_control>(
        "/joy_control", 
        10, 
        [&robot_info, &gait_info](const custom_msgs::Joy_control::ConstPtr& msg) {
            robot_info.run_flag = msg->run_flag;
            if(msg->run_flag == true)
            {
                // 1. 存储线速度（机体系）
                robot_info.body_Vel_des = Vector3d(msg->com_vel.x, msg->com_vel.y, 0.0);
                
                // 2. 存储角速度（机体系）- 假设 msg->yaw_vel 对应绕 Z 轴角速度
                robot_info.body_omega_des = Vector3d(0.0, 0.0, msg->yaw_vel);

                // 3.  Z 期望高度 z_des 
                robot_info.z_des = msg->z_des;
                

                // 5. 读取 mode 并转换为步态枚举存储
                // 假设你的枚举强制转换是安全的
                gait_info.Gait_des = static_cast<Gait_type>(msg->mode);
                if(gait_info.Gait_des!=gait_info.Gait_mode)
                {
                    gait_info.Gait_flag=1;
                }

                //6，mpc是否使用
                robot_info.mpc_use = msg->mpc_use;
            }else 
            {
                robot_info.body_Vel_des.setZero();
                robot_info.body_omega_des.setZero();
            }
            robot_info.world_Vel_des = robot_info.body_Rot_world*robot_info.body_Vel_des;
            robot_info.world_omega_des = robot_info.body_omega_des;
        }
    );
    ros::Subscriber Joy_sub = nh.subscribe<sensor_msgs::Joy>(
        "/joy", 
        100, 
        [&robot_info, &gait_info](const sensor_msgs::Joy::ConstPtr& msg) {
        //rosrun joy joy_node
        //左摇杆对应 msg->axes[0][1] (-1,1)  【2】为左后肩键 按下为-1 不按下为1 
        //右摇杆对应 msg->axes[3][4] (-1,1)  【5】为右后肩键 按下为-1 不按下为1 
        //十字架左右对应 msg->axes[6] 上下 对应 msg->axes[7]

        //A--> msg->buttons[0] B--> msg->buttons[1] Y--> msg->buttons[2] X--> msg->buttons[3]
        //左前肩键 --> msg->buttons[4] 右前肩键 --> msg->buttons[5]
        //左后肩键 --> [6]按下为1 不按下为0 右后肩键 --> [7]按下为1 不按下为0 
        //中间三个键对应msg->buttons[8][9][10]
        //摇杆按下对应msg->buttons[11][12]
        if( msg->buttons[4] == 1 && msg->buttons[5] == 1) //左前肩键和右前肩键同时按下切换运行状态
        {
            robot_info.run_flag = !robot_info.run_flag;
            ROS_WARN("run_flag is changed into %d",robot_info.run_flag);
        }

        double com_vel_x_max = 0.5;
        double com_vel_y_max = 0.3;
        double com_omega_yaw_max = 0.5;
        if(robot_info.run_flag == true)
        {
            // 1. 存储线速度（机体系）左摇杆
            robot_info.body_Vel_des = Vector3d(msg->axes[1]*com_vel_x_max, msg->axes[0]*com_vel_y_max, 0.0);
            
            // 2. 存储角速度（机体系）- 假设 msg->yaw_vel 对应绕 Z 轴角速度 右摇杆
            robot_info.body_omega_des = Vector3d(0.0, 0.0, msg->axes[3]*com_omega_yaw_max);

            // 3.  Z 期望高度 z_des 下降沿检测 十字架
            static int last_Z = msg->axes[7];
            int now_Z = msg->axes[7];
            if( now_Z != 0 && last_Z == 0)
            {
                robot_info.z_des+=0.025*now_Z;
            }
            last_Z = now_Z;
  
            if(msg->buttons[0] == 1) //A
            {
                gait_info.Gait_des = none;
                ROS_WARN("Mode: none");
            }else if(msg->buttons[2] == 1) //Y
            {
                gait_info.Gait_des = stand;
                ROS_WARN("Mode: stand");
            }else if(msg->buttons[3] == 1) //X
            {
                gait_info.Gait_des = walk;
                ROS_WARN("Mode: walk");
            }else if(msg->buttons[1] == 1) //B
            {
                gait_info.Gait_des = trot;
                ROS_WARN("Mode: trot");
            }
            
            if(gait_info.Gait_des!=gait_info.Gait_mode)
            {
                gait_info.Gait_flag=1;
            }

            //6，mpc是否使用
            if(msg->buttons[12] == 1)
            {
                robot_info.mpc_use = !robot_info.mpc_use;
                ROS_WARN("mpc_use Flag: %s", robot_info.mpc_use ? "ON" : "OFF");
            }

        }else 
        {
            robot_info.body_Vel_des.setZero();
            robot_info.body_omega_des.setZero();
        }
        robot_info.world_Vel_des = robot_info.body_Rot_world*robot_info.body_Vel_des;
        robot_info.world_omega_des = robot_info.body_Rot_world*robot_info.body_omega_des;
        
        }
    );
    // **************************
    // 发布节点
    // **************************
    //电机控制节点
    ros::Publisher motor_pub = nh.advertise<custom_msgs::Motor_control>("/Motor_control", 10);
    //Debug节点
    ros::Publisher debug_pub = nh.advertise<custom_msgs::Debug_info>("/Debug_info", 10);



    double thread_dt[4];
    // =========================================================
    // 线程 1  (500Hz) 
    // =========================================================
    std::thread thread_high([&]() {
        // 线程内部获取真实PID（关键！和htop里的PID一致）
        pid_t tid = syscall(SYS_gettid);
        ROS_INFO("[500Hz] PID:%d", tid);

        double target_freq = 500.0;//目标hz
        double expected_cycle_time = 1.0 / target_freq; // 0.002s 目标delta_t
        ros::Rate rate(target_freq);
        #if using_time == 0
            auto last_start_time = std::chrono::steady_clock::now();
            auto current_start_time = std::chrono::steady_clock::now();
            // 运行时间起点
            auto thread_start_time = std::chrono::steady_clock::now();
            double thread_runtime = 0.0;
            // delta_t
            double thread_delta_t_;
        #elif using_time == 1
            // 初始化时间记录 ros::WallTime不会被暂停
            double last_start_time = time_mujoco;
            double current_start_time = time_mujoco;
            //运行时间
            double thread_start_time = time_mujoco;
            double thread_runtime=0.0;
            //delta_t
            double thread_delta_t_;
        #endif
        while (ros::ok()) {
            // double dddt = get_loop_interval();
            // 记录循环开始时间
            #if using_time == 0
                current_start_time = std::chrono::steady_clock::now();
                // 计算差值并转换为秒 (double)，相当于之前的 .toSec()
                thread_delta_t_ = std::chrono::duration<double>(current_start_time - last_start_time).count();
                last_start_time = current_start_time;
                // 计算总运行时间并转换为秒
                thread_runtime = std::chrono::duration<double>(current_start_time - thread_start_time).count();
                thread_dt[0] = thread_delta_t_;
            #elif using_time == 1
                current_start_time = time_mujoco;
                thread_delta_t_ = (current_start_time - last_start_time);
                last_start_time = current_start_time;
                thread_runtime = (current_start_time-thread_start_time);
            #endif
            // ---------------------------------------------
            // 核心控制代码
            // ---------------------------------------------
            //Pinocchio
            dynamics_solver.update(robot_info); 
            //Gait
            Gait_solver.update(gait_info,thread_runtime);
            //Estimate
            if(robot_info.z_des >= 0.1 && Estimate_solver.init_flag == false)//站起来再开始初始化
            {
                Estimate_solver.init_flag = true;
                Estimate_solver.init(robot_info);
                ROS_INFO("Estimate Solver Initialized Successfully.");
            }else if(Estimate_solver.init_flag == true)
            {
                Estimate_solver.update(robot_info,gait_info,0.002);
                robot_info.world_Pos_com = Vector3d(robot_info.X_est[0], robot_info.X_est[1], robot_info.X_est[2]);
                robot_info.world_Vel_com = Vector3d(robot_info.X_est[3],robot_info.X_est[4],robot_info.X_est[5]);
                robot_info.body_Vel_com = robot_info.body_Rot_world.transpose() * robot_info.world_Vel_com;
            }
            //Swing
            Swing_solver.update(swing_info,robot_info,gait_info);
            //Manager
            Manager_solver.update(robot_info,gait_info,swing_info,wbc_info);
            Manager_solver.motor_cmd(robot_info,motor_pub);
            // Wbc
            if(Mpc_solver.mpc_init_flag == true && gait_info.Gait_mode != none )
            {
                Wbc_solver.kin_wbc(robot_info,gait_info,swing_info,0.002);
                Wbc_solver.wbic(robot_info,gait_info,wbc_info);
            }
            // 打印调试信息 (每1秒打印一次，避免刷屏)
            // ROS_INFO_STREAM_THROTTLE(1.0, 
            //     "\n[500Hz Thread]"
            //     << "\n  Rate     : " << std::fixed << std::setprecision(2) << (1.0 / dddt) << " Hz"
            //     << "\n  Delta T  : " << std::setprecision(6) << dddt << " s"
            // );
            
            // 休眠对齐频率
            rate.sleep();
        }
    });
    setThreadCpuAffinity(thread_high,14);
    setThreadPriority(thread_high, 50);
    // =========================================================
    // 线程 2 (200Hz) 
    // =========================================================
    std::thread thread_low([&]() {
        pid_t tid = syscall(SYS_gettid);
        ROS_INFO("[200Hz] PID:%d", tid);
        
        double target_freq = 160.0;//目标hz
        double expected_cycle_time = 1.0 / target_freq; // 0.005s 目标delta_t
        ros::Rate rate(target_freq);
        #if using_time == 0
            auto last_start_time = std::chrono::steady_clock::now();
            auto current_start_time = std::chrono::steady_clock::now();
            // 运行时间起点
            auto thread_start_time = std::chrono::steady_clock::now();
            double thread_runtime = 0.0;
            // delta_t
            double thread_delta_t_;
        #elif using_time == 1
            // 初始化时间记录 ros::WallTime不会被暂停
            double last_start_time = time_mujoco;
            double current_start_time = time_mujoco;
            //运行时间
            double thread_start_time = time_mujoco;
            double thread_runtime=0.0;
            //delta_t
            double thread_delta_t_;
        #endif

        while (ros::ok()) {
            double dddt = get_loop_interval();
            // 记录循环开始时间
            #if using_time == 0
                current_start_time = std::chrono::steady_clock::now();
                // 计算差值并转换为秒 (double)，相当于之前的 .toSec()
                thread_delta_t_ = std::chrono::duration<double>(current_start_time - last_start_time).count();
                last_start_time = current_start_time;
                // 计算总运行时间并转换为秒
                thread_runtime = std::chrono::duration<double>(current_start_time - thread_start_time).count();
                thread_dt[1] = thread_delta_t_;
            #elif using_time == 1
                current_start_time = time_mujoco;
                thread_delta_t_ = (current_start_time - last_start_time);
                last_start_time = current_start_time;
                thread_runtime = (current_start_time-thread_start_time);
            #endif

            // ---------------------------------------------
            // 核心控制代码
            // ---------------------------------------------
            if( Estimate_solver.init_flag == true )//估计结束可以初始化和计算
            {
                if(Mpc_solver.mpc_init_flag == false)
                {
                    Vector13d Q;
                    // roll pitch yaw   x y z          wx wy wz          vx vy vz
                    Q << 10, 10, 10,    1, 1, 80,    0, 0, 0.3,    0.5, 0.5, 1.0, 0;
                    Vector12d R;
                    R.setConstant(5e-6);
                    Mpc_solver.init(robot_info,Q,R);
                    Mpc_solver.mpc_init_flag = true;
                    ROS_INFO("Mpc Solver Initialized Successfully.");
                }else{
                    Mpc_solver.update(robot_info,gait_info,0.00625);
                }
            }
            // ---------------------------------------------
            // 调试
            // ---------------------------------------------

            // // 打印调试信息 (每1秒打印一次，避免刷屏)
            // ROS_INFO_STREAM_THROTTLE(1.0, 
            //     "\n[100Hz Thread]"
            //     << "\n  Rate     : " << std::fixed << std::setprecision(2) << (1.0 / dddt) << " Hz"
            //     << "\n  Delta T  : " << std::setprecision(6) << dddt << " s"
            // );

            // 休眠对齐频率
            rate.sleep();
        }
    });
    setThreadCpuAffinity(thread_low,15);
    // setThreadPriority(thread_low, 60);
    // =========================================================
    // 线程 3 (200Hz)  用于打印调试数据
    // =========================================================
    std::thread thread_debug([&]() {
        ros::Duration(5.0).sleep();
        double target_freq = 100.0;//目标hz
        double expected_cycle_time = 1.0 / target_freq; // 0.01s 目标delta_t
        ros::Rate rate(target_freq);
        #if using_time == 0
            auto last_start_time = std::chrono::steady_clock::now();
            auto current_start_time = std::chrono::steady_clock::now();
            // 运行时间起点
            auto thread_start_time = std::chrono::steady_clock::now();
            double thread_runtime = 0.0;
            // delta_t
            double thread_delta_t_;
        #elif using_time == 1
            // 初始化时间记录 ros::WallTime不会被暂停
            double last_start_time = time_mujoco;
            double current_start_time = time_mujoco;
            //运行时间
            double thread_start_time = time_mujoco;
            double thread_runtime=0.0;
            //delta_t
            double thread_delta_t_;
        #endif

        Rviz_vision rviz_vision(nh, "base_link", "odom");

        while (ros::ok()) {
            // 记录循环开始时间
            #if using_time == 0
                current_start_time = std::chrono::steady_clock::now();
                // 计算差值并转换为秒 (double)，相当于之前的 .toSec()
                thread_delta_t_ = std::chrono::duration<double>(current_start_time - last_start_time).count();
                last_start_time = current_start_time;
                // 计算总运行时间并转换为秒
                thread_runtime = std::chrono::duration<double>(current_start_time - thread_start_time).count();
            #elif using_time == 1
                current_start_time = time_mujoco;
                thread_delta_t_ = (current_start_time - last_start_time);
                last_start_time = current_start_time;
                thread_runtime = (current_start_time-thread_start_time);
            #endif

            // ---------------------------------------------
            // 核心控制代码
            // ---------------------------------------------

            /**************结构体数据调试****************/
            // Debug_robot_info(robot_info,5);
            // Debug_gait_info(gait_info,5);
            // Debug_swing_info(swing_info,5);
            

            /**************mpc调试****************/
            static int mpc_count = 0;
            if (++mpc_count >= (target_freq/10) && Estimate_solver.init_flag == true  ) { //10hz
                // Mpc_solver.debug(robot_info);
                // ROS_INFO("common dt is %lf hz is %lf",thread_dt[0],1/thread_dt[0]);
                // ROS_INFO("mpc dt is %lf hz is %lf",thread_dt[1],1/thread_dt[1]);
                mpc_count = 0;
            }
            /**************wbc调试****************/
            static int wbc_count = 0;
            if (++wbc_count >= (target_freq/10) && Mpc_solver.mpc_init_flag == true ) { //10hz
                // Wbc_solver.debug(wbc_info);
                // Debug_wbc_info(wbc_info,5);
                wbc_count = 0;
            }

            /**************debug调试****************/
            custom_msgs::Debug_info Debug_msg;
            // 1. 质心世界位置 com_pos_world
            Debug_msg.com_pos_world.x = robot_info.world_Pos_com[0];
            Debug_msg.com_pos_world.y = robot_info.world_Pos_com[1];
            Debug_msg.com_pos_world.z = robot_info.world_Pos_com[2];
            // 2. 质心世界速度 com_vel_world
            Debug_msg.com_vel_world.x = robot_info.world_Vel_com[0];
            Debug_msg.com_vel_world.y = robot_info.world_Vel_com[1];
            Debug_msg.com_vel_world.z = robot_info.world_Vel_com[2];
            // 3. 质心机体欧拉角 com_euler_body (Roll, Pitch, Yaw)
            Debug_msg.com_euler_body.x = robot_info.euler[0];
            Debug_msg.com_euler_body.y = robot_info.euler[1];
            Debug_msg.com_euler_body.z = robot_info.euler[2];
            // 4. 质心机体角速度 com_omega_body (wx, wy, wz)
            Debug_msg.com_omega_body.x = robot_info.body_Omega[0];
            Debug_msg.com_omega_body.y = robot_info.body_Omega[1];
            Debug_msg.com_omega_body.z = robot_info.body_Omega[2];
            debug_pub.publish(Debug_msg);

            /**************rviz显示调试****************/
            static int rviz_count = 0;
            static std::vector<double> joint_angles_vec(12);
            if (++rviz_count >= (target_freq/50)) { // 100Hz / 2 = 50Hz，足够平滑了
                for (int i = 0; i < 12; ++i) {
                    joint_angles_vec[i] = robot_info.Pos_motor[i];
                    if(i == 1 || i == 4 || i == 7 || i == 10) //关节2偏移45°
                    {
                        joint_angles_vec[i] -=0.785;
                    }
                    if(i == 2 || i == 5 || i == 8 || i == 11) //关节3偏移-135°
                    {
                        joint_angles_vec[i] +=2.355;
                    }
                }
                rviz_vision.updateRobotState(robot_info.world_Pos_com,robot_info.quat_base,joint_angles_vec);
                for(int i=0;i<4;i++)
                {
                    //起点 (绿色) 中点 (蓝色) 落足点 (红色)
                    rviz_vision.visualizeSwingLeg(swing_info.world_POS_start_touch.col(i),
                                                swing_info.world_POS_mid_touch.col(i),
                                                swing_info.world_POS_end_touch.col(i),
                                                i);
                }
                rviz_vision.point_to_link(swing_info.link1_POS_foot.col(0),"leg1_link1",30);
                rviz_vision.point_to_link(swing_info.link1_POS_foot.col(1),"leg2_link1",31);
                rviz_vision.point_to_link(swing_info.link1_POS_foot.col(2),"leg3_link1",32);
                rviz_vision.point_to_link(swing_info.link1_POS_foot.col(3),"leg4_link1",33);

                // rviz_vision.point_to_link(swing_info.world_POS_foot.col(0),"odom",30);
                // rviz_vision.point_to_link(swing_info.world_POS_foot.col(1),"odom",31);
                // rviz_vision.point_to_link(swing_info.world_POS_foot.col(2),"odom",32);
                // rviz_vision.point_to_link(swing_info.world_POS_foot.col(3),"odom",33);

                rviz_count = 0;
            }

            // 休眠对齐频率
            rate.sleep();
        }
    });
    // setThreadCpuAffinity(thread_debug,13);
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
    if (thread_debug.joinable()) thread_debug.join();

    ROS_INFO("Main Node Exited Cleanly.");
    return 0;
}

double get_loop_interval(void) 
{
    // 使用 static 变量存储上一次的时间点，函数结束后不会被销毁
    static auto last_time = std::chrono::steady_clock::now();
    static bool first_run = true;

    auto current_time = std::chrono::steady_clock::now();
    
    // 如果是第一次运行，初始化时间并返回 0
    if (first_run) {
        last_time = current_time;
        first_run = false;
        return 0.0;
    }

    // 计算差值
    std::chrono::duration<double> diff = current_time - last_time;
    double delta_t = diff.count();

    // 更新静态变量，供下一次循环使用
    last_time = current_time;

    // 防御性处理，防止由于高频调用导致的极小值或 0
    return (delta_t > 0.0) ? delta_t : 1e-6; 
}

bool setThreadCpuAffinity(std::thread& thread, int cpu_core) {
    // 检查核心编号是否合法（比如4核CPU，核心编号0-3）
    int cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu_core < 0 || cpu_core >= cpu_count) {
        ROS_ERROR("CPU num error! system has %d core with 0-%d", cpu_count, cpu_count-1);
        return false;
    }

    // 构造 CPU 核心掩码（只允许指定核心）
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);       // 清空掩码
    CPU_SET(cpu_core, &cpuset); // 把指定核心加入掩码

    // 设置线程的 CPU 亲密性
    pthread_t thread_handle = thread.native_handle();
    int ret = pthread_setaffinity_np(thread_handle, sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        ROS_ERROR("set CPU error! code : %d", ret);
        return false;
    }

    ROS_INFO("set CPU %d success ", cpu_core);
    return true;
}

bool setThreadPriority(std::thread& thread, int priority) {
    sched_param sch;
    sch.sched_priority = priority;
    pthread_t thread_handle = thread.native_handle();
    
    // SCHED_FIFO 是一种实时调度策略，通常用于要求高响应的控制线程
    // 注意：在普通用户权限下，这可能会失败（返回非0错误码）
    int ret = pthread_setschedparam(thread_handle, SCHED_FIFO, &sch);
    if (ret != 0) {
        ROS_ERROR("Set thread priority error! code : %d", ret);
        return false;
    }

    ROS_INFO("Set thread priority to %d success", priority);
    return true;
}
