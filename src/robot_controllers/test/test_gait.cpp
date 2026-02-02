
#include "Common.h" //  Common.h
#include "Gait.h"// 包含 Gait 类
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
        

using namespace controllers;

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_gait_node");
    ros::NodeHandle nh;
    ros::Publisher pub_debug = nh.advertise<std_msgs::Float64MultiArray>("gait_debug", 10);
    ros::Rate rate(1000); // 1kHz 控制频率

    // 1. 实例化
    // 注意：请确保 Gait 类的构造函数是 public 的，否则这里无法实例化
    Gait gait_scheduler; 
    Gait_info gait_data;

    // 初始化为 Trot
    gait_data.Gait_des = Gait_type::trot;
    // 手动调用一次 change_mode 初始化参数 (模拟构造函数行为)或者依赖默认值
    // 这里为了严谨，手动赋初值
    gait_data.time_gait = 0.6;
    gait_data.Gait_ratio.setConstant(0.5);
    gait_data.Gait_offset << 0.5, 0.0, 0.0, 0.5;

    double current_time = 0.0;
    double dt = 0.001;
    bool switch_command_sent = false;

    ROS_INFO("Start Simulation: TROT -> WALK switching test...");

    while (ros::ok()) {
        // ==========================================
        // 模拟外部指令输入
        // ==========================================
        
        // 在第 2.0 秒发送切换指令
        if (current_time > 2.0 && !switch_command_sent) {
            ROS_WARN("[CMD] Requesting switch to WALK at time: %.4f", current_time);
            gait_data.Gait_flag = 1;         // 举起切换旗帜
            gait_data.Gait_des = Gait_type::walk; // 目标：Walk
            switch_command_sent = true;
        }

        // ==========================================
        // 记录切换前的数据 (用于对比)
        // ==========================================
        double prev_period = gait_data.time_gait;
        double prev_phase = gait_data.time_gait_degree;

        // ==========================================
        // 核心更新
        // ==========================================
        gait_scheduler.update(gait_data, current_time);

        // ==========================================
        // 检测切换是否发生 (Check Transition)
        // ==========================================
        // 如果周期变了，或者 flag 被复位了(需要在change_mode里复位)，说明切换发生了
        // 这里我们通过观察 flag 是否自动归零来判断切换（如果你修改了update代码）
        // 或者通过观察周期变化
        if (switch_command_sent && gait_data.time_gait != prev_period) {
            ROS_ERROR(">>> GAIT SWITCHED! <<<");
            ROS_INFO("Trigger Time: %.4f", current_time);
            ROS_INFO("Old Phase: %.4f -> New Phase: %.4f (Should be near 0)", prev_phase, gait_data.time_gait_degree);
            ROS_INFO("Old Period: %.4f -> New Period: %.4f", prev_period, gait_data.time_gait);
            
            // 验证是否在 0.95 附近触发
            if (prev_phase < 0.95) {
                ROS_ERROR("FAIL: Switched too early! Phase was %.4f", prev_phase);
            } else {
                ROS_INFO("PASS: Switched at correct phase window.");
            }
        }

        // ==========================================
        // 可视化数据发布 (rqt_plot)
        // ==========================================
        std_msgs::Float64MultiArray msg;
        // layout: [0]=global_phase, [1]=LF_swing_degree, [2]=LF_state, [3]=Period
        msg.data.push_back(gait_data.time_gait_degree);
        msg.data.push_back(gait_data.Time_swing_degree[0]); // 左前腿摆动进度
        msg.data.push_back(gait_data.Gait_state[0]);        // 左前腿接触状态
        msg.data.push_back(gait_data.time_gait);            // 周期
        pub_debug.publish(msg);

        // 时间推进
        current_time += dt;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}