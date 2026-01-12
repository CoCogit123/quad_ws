#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h> //gazebo/clock

//消息包
#include <custom_msgs/Motor_state.h>
#include <custom_msgs/Motor_control.h>
#include <custom_msgs/Sim_info.h>
custom_msgs::Motor_state Motor_state_msgs;
custom_msgs::Motor_control Motor_control_msgs;
custom_msgs::Sim_info Sim_info_msgs;
void msgs_init()
{
    Motor_state_msgs.id.resize(12);
    Motor_state_msgs.pos.resize(12);
    Motor_state_msgs.vel.resize(12);
    Motor_state_msgs.torque.resize(12);
    for(int i=0;i<12;i++)
    {
        Motor_state_msgs.id[i] = i;
    }
    
    Motor_control_msgs.id.resize(12);
    Motor_control_msgs.torque.resize(12);
    Motor_control_msgs.kp.resize(12);
    Motor_control_msgs.kd.resize(12);
    Motor_control_msgs.pos.resize(12);
    Motor_control_msgs.vel.resize(12);
}
void motor_control_callback(const custom_msgs::Motor_control &msg)
{
    Motor_control_msgs=msg;
}


//gazebo回传电机数据
void motor_rev_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    static double leg1_offest=0.00;
    static double leg2_offest=0.00;
    static double leg3_offest=0.00;
    std::memcpy(Motor_state_msgs.pos.data(), msg->position.data(), msg->position.size()* sizeof(double));
    std::memcpy(Motor_state_msgs.vel.data(), msg->velocity.data(), msg->velocity.size()* sizeof(double));
    std::memcpy(Motor_state_msgs.torque.data(), msg->effort.data(), msg->effort.size()* sizeof(double));
    //偏移回标准坐标系
    Motor_state_msgs.pos[0]+=leg1_offest; Motor_state_msgs.pos[3]-=leg1_offest; Motor_state_msgs.pos[6]+=leg1_offest; Motor_state_msgs.pos[9]-=leg1_offest;
    Motor_state_msgs.pos[1]+=leg2_offest; Motor_state_msgs.pos[4]+=leg2_offest; Motor_state_msgs.pos[7]+=leg2_offest; Motor_state_msgs.pos[10]+=leg2_offest;
    Motor_state_msgs.pos[2]+=leg3_offest; Motor_state_msgs.pos[5]+=leg3_offest; Motor_state_msgs.pos[8]+=leg3_offest; Motor_state_msgs.pos[11]+=leg3_offest;
}
//gazebo回传仿真运行时间
double sim_time_sec = 0.0;
void clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
    // 1. 获取ros::Time类型的仿真时间
    ros::Time sim_time = msg->clock;
    
    // 2. 转换为秒（double类型，包含小数部分）
    sim_time_sec = sim_time.toSec();   
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"sim_gazebo");
    ros::NodeHandle nh;
    msgs_init();

    ros::Publisher state_pub =nh.advertise<custom_msgs::Motor_state>("/Motor_state",10);//发送出去电机数据
    ros::Subscriber control_sub=nh.subscribe("/Motor_control",10,motor_control_callback);//接收控制电机数据
    ros::Publisher sim_info_pub =nh.advertise<custom_msgs::Sim_info>("/Sim_info",10);//发送出去sim_info
    ros::Subscriber clock_sub = nh.subscribe("/clock", 10, clock_callback);//订阅获取仿真时间
    /*********gazebo仿真要的***********/
    ros::Publisher cmd_pub[12];
    //初始化各个电机话题
    cmd_pub[0]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg1_1/command",10);
    cmd_pub[1]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg1_2/command",10);
    cmd_pub[2]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg1_3/command",10);

    cmd_pub[3]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg2_1/command",10);
    cmd_pub[4]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg2_2/command",10);
    cmd_pub[5]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg2_3/command",10);

    cmd_pub[6]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg3_1/command",10);
    cmd_pub[7]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg3_2/command",10);
    cmd_pub[8]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg3_3/command",10);

    cmd_pub[9]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg4_1/command",10);
    cmd_pub[10]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg4_2/command",10);
    cmd_pub[11]=nh.advertise<std_msgs::Float64>("/dog_controllers/leg4_3/command",10);
    //接收电机状态包 从gazebo来的
    ros::Subscriber rev_sub = nh.subscribe("/joint_states",20,motor_rev_callback);
    //必须以此类型消息包控制 用于自定义消息包和gazebo仿真中转
    std_msgs::Float64 cmd_msg_std[12];

    ros::Rate rate1(1000);//设置循环频率
    while(ros::ok())
    {
        ros::spinOnce();//回调接收控制参数
        //发送电机控制包
        for(int i =0;i<12;i++)
        {
            //确保已经初始化了且发送顺序对 0-11
            if(Motor_control_msgs.id[i] == i)
            {
                cmd_msg_std[i].data = Motor_control_msgs.torque[i]
                                + Motor_control_msgs.kp[i] * ( Motor_control_msgs.pos[i] - Motor_state_msgs.pos[i]) 
                                + Motor_control_msgs.kd[i] * ( Motor_control_msgs.vel[i] - Motor_state_msgs.vel[i]);;
                cmd_pub[i].publish(cmd_msg_std[i]);
            } else {
                cmd_msg_std[i].data = 0;
                cmd_pub[i].publish(cmd_msg_std[i]);
            }
        }
        // ROS_INFO("diceng_torque: %f %f",cmd_msg_.torque[1],cmd_msg_std[1].data);
        //发送电机状态包
        state_pub.publish(Motor_state_msgs);
        //sim_info
        Sim_info_msgs.run_time = sim_time_sec;
        sim_info_pub.publish(Sim_info_msgs);
        rate1.sleep();

    }
    

    /* code */
    return 0;
}