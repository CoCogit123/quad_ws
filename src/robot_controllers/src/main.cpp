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
#include "Rviz_vision.h"
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
#include <sensor_msgs/Joy.h> //joy消息包

using namespace controllers;//自定义工作空间
double get_loop_interval(void);

#define using_time 1  //0：使用不停止的接近现实时间  1：使用mujoco时间
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
            robot_info.world_Pos_com = Vector3d(msg->sim_pos.x, msg->sim_pos.y, msg->sim_pos.z);
            robot_info.world_Pos_com[2]-=0.0189;//减去足端半径 默认足端圆心为地面0处
            robot_info.body_Vel_com = Vector3d(msg->sim_twist.linear.x,msg->sim_twist.linear.y,msg->sim_twist.linear.z);
            robot_info.world_Vel_com = robot_info.body_Rot_world*robot_info.body_Vel_com;
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
                
                static ros::Time last_time = ros::Time::now(); // 仅在第一次调用时初始化
                ros::Time current_time = ros::Time::now();
                // 计算两次回调之间的实际时间差
                double dt = (current_time - last_time).toSec();
                // 更新时间戳供下次使用
                last_time = current_time;
                // --- 鲁棒性保护 ---
                // 第一次运行或间隔过长（比如程序卡住后恢复）时，给一个合理的默认值
                if (dt <= 0.0 || dt > 0.5) {
                    dt = 0.02; // 假设期望频率是 50Hz，则设为 0.02s
                }
                robot_info.euler_des[2] = robot_info.euler_des[2] + msg->yaw_vel*dt;

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

        double com_vel_x_max = 0.6;
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

            static double last_yaw = 0;
            double now_yaw = msg->axes[3];
            static ros::Time trigger_time = ros::Time::now();
            if( std::fabs(now_yaw) >= 0.005 ) 
            {
                if(std::fabs(last_yaw) <= 0.005) { trigger_time = ros::Time::now(); }//记录下降沿触发时间
                int total_duration = (int)((ros::Time::now() - trigger_time).toSec()*1000);//转化成ms
                if(total_duration>=10)//每隔10ms触发一次 100hz
                {
                    robot_info.euler_des[2] += msg->axes[3]*com_omega_yaw_max*0.01; //速度乘以时间
                    trigger_time = ros::Time::now();
                }
            }
            last_yaw = now_yaw;
            
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
    ros::Publisher motor_pub = nh.advertise<custom_msgs::Motor_control>("/Motor_control", 10);
    // =========================================================
    // 线程 1  (500Hz) 
    // =========================================================
    std::thread thread_high([&]() {
        double target_freq = 500.0;//目标hz
        double expected_cycle_time = 1.0 / target_freq; // 0.002s 目标delta_t
        ros::Rate rate(target_freq);
        #if using_time == 0
            // 初始化时间记录 ros::WallTime不会被暂停
            ros::WallTime last_start_time = ros::WallTime::now();
            ros::WallTime current_start_time = ros::WallTime::now();
            //运行时间
            ros::WallTime thread_start_time = ros::WallTime::now();
            double thread_runtime=0.0;
            //delta_t
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
                current_start_time = ros::WallTime::now();
                thread_delta_t_ = (current_start_time - last_start_time).toSec();
                last_start_time = current_start_time;
                thread_runtime = (current_start_time-thread_start_time).toSec();
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
            }
            //Swing
            Swing_solver.update(swing_info,robot_info,gait_info);
            //Manager
            Manager_solver.update(robot_info,gait_info,swing_info);
            Manager_solver.motor_cmd(robot_info,motor_pub);


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

    // =========================================================
    // 线程 2 (200Hz) 
    // =========================================================
    std::thread thread_low([&]() {
        double target_freq = 200.0;//目标hz
        double expected_cycle_time = 1.0 / target_freq; // 0.005s 目标delta_t
        ros::Rate rate(target_freq);
        #if using_time == 0
            // 初始化时间记录 ros::WallTime不会被暂停
            ros::WallTime last_start_time = ros::WallTime::now();
            ros::WallTime current_start_time = ros::WallTime::now();
            //运行时间
            ros::WallTime thread_start_time = ros::WallTime::now();
            double thread_runtime=0.0;
            //delta_t
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
                current_start_time = ros::WallTime::now();
                thread_delta_t_ = (current_start_time - last_start_time).toSec();
                last_start_time = current_start_time;
                thread_runtime = (current_start_time-thread_start_time).toSec();
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
                    // roll pitch yaw  x y z          wx wy wz          vx vy vz
                    Q << 50, 50, 10,   10, 10, 200,    0.05, 0.05, 0.3,    0.5, 0.5, 10.0,   0; 
                    Vector12d R;
                    R.setConstant(0.00001);//alpha > 1e-4过高，建议调整到1e-5
                    Mpc_solver.init(robot_info,Q,R);
                    Mpc_solver.mpc_init_flag = true;
                    ROS_INFO("Mpc Solver Initialized Successfully.");
                }else{
                    Mpc_solver.update(robot_info,gait_info,0.005);
                }
            }

            // ---------------------------------------------
            // 调试
            // ---------------------------------------------
            static ros::Time last_print_time = ros::Time::now(); // 静态变量，只初始化一次
            ros::Time now = ros::Time::now();
            // 检查时间间隔是否超过 0.1 秒
            if ((now - last_print_time).toSec() >= 1 && Estimate_solver.init_flag == true ) { // 
                
                // 安全检查
                if (Mpc_solver.qp_solution.size() >= 12) {
                    // 提取前 12 个元素，避免重复调用 head()
                    Eigen::VectorXd f = Mpc_solver.qp_solution.head(12);

                    // 设置打印格式：固定小数点，保留3位
                    std::cout << std::fixed << std::setprecision(3);

                    // 打印表头 (时间 + 腿名称)
                    std::cout << "\n\033[1;33m[QP ] " << "\033[0m" << std::endl;
                    std::cout << "      | " 
                            << std::setw(9) << "FL" << " | " 
                            << std::setw(9) << "FR" << " | " 
                            << std::setw(9) << "RL" << " | " 
                            << std::setw(9) << "RR" << " |" << std::endl;
                    std::cout << "-------------------------------------------------" << std::endl;

                    // 定义行名
                    const char* axis_names[3] = {"Fx (N)", "Fy (N)", "Fz (N)"};

                    // 循环打印 3 行 (Fx, Fy, Fz)
                    for (int axis = 0; axis < 3; ++axis) {
                        std::cout << std::setw(5) << axis_names[axis] << " | ";
                        
                        // 循环打印 4 列 (FL, FR, RL, RR)
                        for (int leg = 0; leg < 4; ++leg) {
                            // 索引逻辑：第 leg 条腿的第 axis 分量
                            // 假设排列为 [FLx, FLy, FLz, FRx, FRy, FRz ...]
                            int index = leg * 3 + axis; 
                            
                            double val = f(index);
                            
                            // 根据数值正负设置颜色 (可选：正数绿色，负数红色，0灰色)
                            if(std::abs(val) < 0.001) std::cout << "\033[90m"; // 灰色
                            else if(val >= 0)         std::cout << "\033[32m"; // 绿色
                            else                      std::cout << "\033[31m"; // 红色

                            std::cout << std::setw(9) << val << "\033[0m | ";
                        }
                        std::cout << std::endl;
                    }
                    std::cout << "=================================================" << std::endl;

                } else {
                    std::cout << "[QP Result] Error: Solution size too small (" 
                            << Mpc_solver.qp_solution.size() << ")" << std::endl;
                }

                // 定义状态名称 (根据 MPC 标准 13 维状态)
                const char* state_names[13] = {
                    "Roll (rad) ", "Pitch (rad)", "Yaw (rad)  ",  // 6-8 (或者欧拉角)
                    "Pos X (m)  ", "Pos Y (m)  ", "Pos Z (m)  ",  // 0-2
                    "Omg X (r/s)", "Omg Y (r/s)", "Omg Z (r/s)",  // 9-11
                    "Vel X (m/s)", "Vel Y (m/s)", "Vel Z (m/s)",  // 3-5
                    "Gravity (g)"                                  // 12
                };

                std::cout << "\n\033[1;36m[State Tracking] x_now vs X_des(k=0)\033[0m" << std::endl;
                std::cout << "Idx | Name        |     x_now |     X_des |      Diff |" << std::endl;
                std::cout << "--------------------------------------------------------" << std::endl;

                // 遍历 13 个状态
                for (int i = 0; i < 13; ++i) {
                    double val_now = Mpc_solver.x_now(i);
                    // 安全检查：防止 X_des 为空
                    double val_des = Mpc_solver.X_des(i);

                    if(i>=0&&i<=2) { val_now*=57.32; val_des*=57.32;}
                    double diff = val_des - val_now;

                    // 设置每一行的颜色：
                    // 如果误差很小(灰色)，误差中等(白色)，误差大(红色/黄色)
                    std::string color = "\033[0m"; // 默认白色
                    if (std::abs(diff) < 0.01) color = "\033[90m";       // 灰色 (Tracking很好)
                    else if (std::abs(diff) > 0.1) color = "\033[1;31m"; // 红色加粗 (偏差大)
                    else if (std::abs(diff) > 0.05) color = "\033[33m";  // 黄色 (有偏差)

                    std::cout << std::fixed << std::setprecision(4);
                    
                    std::cout << std::setw(3) << i << " | " 
                            << state_names[i] << " | " 
                            << color
                            << std::setw(9) << val_now << " | " 
                            << std::setw(9) << val_des << " | " 
                            << std::setw(9) << diff << "\033[0m |" << std::endl;
                }
                ROS_INFO("z_des is %f",robot_info.z_des);
                std::cout << "========================================================" << std::endl;

                last_print_time = now;
            }

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

    // =========================================================
    // 线程 3 (200Hz)  用于打印调试数据
    // =========================================================
    std::thread thread_debug([&]() {
        ros::Duration(5.0).sleep();
        double target_freq = 200.0;//目标hz
        double expected_cycle_time = 1.0 / target_freq; // 0.01s 目标delta_t
        ros::Rate rate(target_freq);
        #if using_time == 0
            // 初始化时间记录 ros::WallTime不会被暂停
            ros::WallTime last_start_time = ros::WallTime::now();
            ros::WallTime current_start_time = ros::WallTime::now();
            //运行时间
            ros::WallTime thread_start_time = ros::WallTime::now();
            double thread_runtime=0.0;
            //delta_t
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
                current_start_time = ros::WallTime::now();
                thread_delta_t_ = (current_start_time - last_start_time).toSec();
                last_start_time = current_start_time;
                thread_runtime = (current_start_time-thread_start_time).toSec();
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
            // Debug_robot_info(robot_info,1);
            // Debug_gait_info(gait_info,5);
            // Debug_swing_info(swing_info,5);
            
            /**************rviz显示调试****************/
            static int rviz_count = 0;
            
            static std::vector<double> joint_angles_vec(12);
            if (++rviz_count >= 4) { // 200Hz / 4 = 50Hz，足够平滑了
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
            }

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
    if (thread_debug.joinable()) thread_debug.join();

    ROS_INFO("Main Node Exited Cleanly.");
    return 0;
}

double get_loop_interval(void) {
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