#ifndef MANAGER_H
#define MANAGER_H

#include "Common.h"
#include "Dynamics.h"
#include <ros/ros.h>
#include <custom_msgs/Motor_state.h>

namespace controllers {

class Manager{
    public:Manager(){}
        void update(Robot_info &robot,Gait_info &gait,Swing_info &swing)
        {
            if(gait.Gait_mode == none)
            {
                damping_control(robot);
            }else
            {
                //swing
                swing_torque_control(robot,gait,swing);
                //stand
                stand_pos_control(robot,gait); 
            }
        }
        void motor_cmd(const Robot_info& robot, ros::Publisher& pub) {
            custom_msgs::Motor_control msg;

            // 预留 12 个电机的空间
            msg.id.resize(12);
            msg.torque.resize(12);
            msg.kp.resize(12);
            msg.kd.resize(12);
            msg.pos.resize(12);
            msg.vel.resize(12);
            if(robot.run_flag == true && robot.safe_flag == true)
            {
                for (int i = 0; i < 12; ++i) {
                msg.id[i] = static_cast<int16_t>(i); // 电机ID通常为 0-11
                msg.pos[i] = robot.Pos_motor_cmd(i);
                msg.vel[i] = robot.Vel_motor_cmd(i);
                msg.kp[i]  = robot.Kp_motor(i);
                msg.kd[i]  = robot.Kd_motor(i);
                msg.torque[i] = robot.Torque_motor(i);
                }
            }else{
                for (int i = 0; i < 12; ++i) {
                msg.id[i] = static_cast<int16_t>(i); // 电机ID通常为 0-11
                msg.pos[i] = 0;
                msg.vel[i] = 0;
                msg.kp[i]  = 0;
                msg.kd[i]  = 3;
                msg.torque[i] = 0;
                }
            }
            
            pub.publish(msg);
        }
    private:
        double kp_cmd = 60;
        double kd_cmd = 0.7;
        /**
         * @brief 摆动相位置控制
         * @param 获取参数 计算Robot_info的最终电机控制参数
         */
        void swing_pos_control(Robot_info &robot,Gait_info &gait,Swing_info &swing)
        {
            for(int i=0;i<4;i++)
            {
                if(gait.Gait_state[i]==0)
                {
                    robot.Pos_motor_cmd.segment<3>(i*3) = Dynamics::inverse_kinematic(swing.link1_POS_foot.col(i),robot,1,i);
                    robot.Vel_motor_cmd.segment<3>(i*3).setZero();
                    robot.Kp_motor.segment<3>(i*3).setConstant(kp_cmd);
                    robot.Kd_motor.segment<3>(i*3).setConstant(kd_cmd);
                    robot.Torque_motor.segment<3>(i*3).setZero();
                }
            }
        }
        /**
         * @brief 摆动相力矩控制（）
         * @param 获取参数 计算Robot_info的最终电机控制参数
         */
        void swing_torque_control(Robot_info &robot,Gait_info &gait,Swing_info &swing)
        {
            for(int i=0;i<4;i++)
            {
                if(gait.Gait_state[i]==0)
                {
                    robot.Pos_motor_cmd.segment<3>(i*3).setZero();
                    robot.Vel_motor_cmd.segment<3>(i*3).setZero();
                    robot.Kp_motor.segment<3>(i*3).setZero();
                    robot.Kd_motor.segment<3>(i*3).setZero();
                    robot.Torque_motor.segment<3>(i*3).setZero();
                }
            }
        }
        /**
         * @brief 支撑相位置控制
         * @param 获取参数 计算Robot_info的最终电机控制参数
         */
        void stand_pos_control(Robot_info &robot,Gait_info &gait)
        {
            for(int i=0;i<4;i++)
            {
                if(gait.Gait_state[i]==1)
                {
                    Vector3d pos = Dynamics::posture_to_footpos(robot,i);
                    robot.Pos_motor_cmd.segment<3>(i*3) = Dynamics::inverse_kinematic(pos,robot,1,i);
                    robot.Vel_motor_cmd.segment<3>(i*3).setZero();
                    robot.Kp_motor.segment<3>(i*3).setConstant(kp_cmd);
                    robot.Kd_motor.segment<3>(i*3).setConstant(kd_cmd);
                    robot.Torque_motor.segment<3>(i*3).setZero();
                }
            }
        }

        /**
         * @brief 阻尼控制
         * @param 
         */
        void damping_control(Robot_info &robot)
        {
            for(int i=0;i<4;i++)
            {
                robot.Pos_motor_cmd.segment<3>(i*3).setZero();
                robot.Vel_motor_cmd.segment<3>(i*3).setZero();
                robot.Kp_motor.segment<3>(i*3).setZero();
                robot.Kd_motor.segment<3>(i*3).setConstant(3);
                robot.Torque_motor.segment<3>(i*3).setZero();
            }
        }
};
}
#endif 