
#include "Estimate.h" // 确保包含您的头文件
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>


using namespace controllers;

int main(int argc, char** argv) {
    ros::init(argc, argv, "estimation_test_node");
    ros::NodeHandle nh;

    // 创建估计器实例
    Estimate estimator;
    
    // 模拟 Robot_info 数据
    Robot_info robot_data;
    double dt = 0.002; // 500Hz
    robot_data.rot_mat = Eigen::Matrix3d::Identity();
    robot_data.motor_q.setZero();
    for(int i=00;i<4;i++)
    {
        robot_data.foot_pos_body[i].setZero();
        robot_data.foot_vel_body[i].setZero();
    }
    robot_data.imu_acc = Eigen::Vector3d::Zero(); // 假设已去重力
    
    // 初始足端位置（假设机器人站立在 0.6m 高处）
    // FR, FL, RR, RL
    robot_data.foot_pos_body[0] << 0.2,-0.15,-0.6;
    robot_data.foot_pos_body[1] << 0.2,0.15,-0.6;
    robot_data.foot_pos_body[2] << -0.2,-0.15,-0.6;
    robot_data.foot_pos_body[3] << -0.2,0.15,-0.6;
                                
    // 初始化估计器
    estimator.init(robot_data);

    // 发布估计结果
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/state_estimation/odom", 10);
    
    ros::Rate rate(500);
    while (ros::ok()) {
        // 模拟：让机器人以 0.1m/s 沿 X 轴移动
        // 在支撑相，足端相对于机体的速度应该是 -0.1m/s
        for(int i=0; i<4; ++i) {
            robot_data.contact_state[i] = 1; // 假设全支撑
            robot_data.foot_vel_body[i][2] = -0.1; 
        }

        // 调用更新
        estimator.update(robot_data,dt);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}