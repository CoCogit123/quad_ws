#include <thread>
#include <iostream>
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Float64.h>

#include "UI_interface.h"
#include "MJ_interface.h"

//消息包
#include <custom_msgs/Motor_state.h>
#include <custom_msgs/Motor_control.h>
#include <custom_msgs/Sim_info.h>
#include <sensor_msgs/Imu.h>
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

// MuJoCo load and compile model
mjModel* mj_model;
mjData* mj_data;

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"sim_mujoco");
    ros::NodeHandle nh("~");
    msgs_init();

    char error[1000] = "Could not load binary model";
    std::string now_pkg_dir = ros::package::getPath("robot_description");//模型所在包路径
    std::string xml_path = now_pkg_dir + "/robot/xml/" + "robot_scene.xml";//xml路径
    
    mj_model = mj_loadXML(xml_path.c_str(), 0, error, 1000);
    mj_data = mj_makeData(mj_model);
    UIctr uiController(mj_model,mj_data);
    MJ_Interface mj_interface(mj_model, mj_data);
    // init UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false);
    mjtNum simstart = mj_data->time;
    mju_copy(mj_data->qpos,mj_model->key_qpos,mj_model->nq*1);//固定住起始姿态

    //和外部的
    double start_time = mj_data->time;
    double run_time;
    std::cout << mj_model->opt.timestep << std::endl;
    ros::Publisher state_pub =nh.advertise<custom_msgs::Motor_state>("/Motor_state",10);//发送出去电机数据
    ros::Subscriber control_sub=nh.subscribe("/Motor_control",10,motor_control_callback);//接收控制电机数据
    ros::Publisher sim_info_pub =nh.advertise<custom_msgs::Sim_info>("/Sim_info",10);//发送出去sim_info
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 10);//imu发送
    ros::Rate rate(1000);
    while(ros::ok())
    {
        //仿真部分
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim)
        {
            mj_step(mj_model, mj_data);
            mj_interface.updateSensorValues();

            /******************和外部的******************/
            ros::spinOnce();//回调接收控制参数
            //发送出去电机数据
            for (size_t i = 0; i < mj_interface.motor_pos.size(); ++i) 
            {
                //ros消息包
                Motor_state_msgs.pos[i] = mj_interface.motor_pos[i];
                Motor_state_msgs.vel[i] = mj_interface.motor_vel[i];
            }
            state_pub.publish(Motor_state_msgs);
            //接收控制电机数据
            double true_torque[12];
            for(int i=0;i<12;i++)
            {
                true_torque[i] = Motor_control_msgs.torque[i] 
                                + Motor_control_msgs.kp[i] * ( Motor_control_msgs.pos[i] - Motor_state_msgs.pos[i]) 
                                + Motor_control_msgs.kd[i] * ( Motor_control_msgs.vel[i] - Motor_state_msgs.vel[i]);
            }
            std::vector<double> send_torque;
            for(size_t i = 0; i < mj_interface.motor_pos.size(); ++i)
            {
                send_torque.push_back(true_torque[i]);
            }
            mj_interface.setMotorsTorque(send_torque);
            //Sim_info
            run_time = mj_data->time - start_time;
            Sim_info_msgs.run_time = run_time;
            Sim_info_msgs.sim_pos.x = mj_interface.basePos[0];
            Sim_info_msgs.sim_pos.y = mj_interface.basePos[1];
            Sim_info_msgs.sim_pos.z = mj_interface.basePos[2];

            Sim_info_msgs.sim_quat.x = mj_interface.baseQuat[0];
            Sim_info_msgs.sim_quat.y = mj_interface.baseQuat[1];
            Sim_info_msgs.sim_quat.z = mj_interface.baseQuat[2];
            Sim_info_msgs.sim_quat.w = mj_interface.baseQuat[3];

            Sim_info_msgs.sim_twist.linear.x = mj_interface.baseLinVel[0];
            Sim_info_msgs.sim_twist.linear.y = mj_interface.baseLinVel[1];
            Sim_info_msgs.sim_twist.linear.z = mj_interface.baseLinVel[2];

            Sim_info_msgs.sim_twist.angular.x = mj_interface.baseAngVel[0];
            Sim_info_msgs.sim_twist.angular.y = mj_interface.baseAngVel[1];
            Sim_info_msgs.sim_twist.angular.z = mj_interface.baseAngVel[2];
            sim_info_pub.publish(Sim_info_msgs);
            //imu
            sensor_msgs::Imu imu_msg;
            // 设置消息头
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "base_link";
            imu_msg.orientation.x = mj_interface.baseQuat[0];
            imu_msg.orientation.y = mj_interface.baseQuat[1];
            imu_msg.orientation.z = mj_interface.baseQuat[2];
            imu_msg.orientation.w = mj_interface.baseQuat[3];
            imu_msg.angular_velocity.x = mj_interface.baseAngVel[0];
            imu_msg.angular_velocity.y = mj_interface.baseAngVel[1];
            imu_msg.angular_velocity.z = mj_interface.baseAngVel[2];
            imu_msg.linear_acceleration.x = mj_interface.baseAcc[0];
            imu_msg.linear_acceleration.y = mj_interface.baseAcc[1];
            imu_msg.linear_acceleration.z = mj_interface.baseAcc[2];
            imu_pub.publish(imu_msg);
        }

        uiController.updateScene();
    }

    uiController.Close();
    return 0;
}