#ifndef MANAGER_H
#define MANAGER_H

#include "Common.h"
#include "Dynamics.h"
#include <ros/ros.h>


namespace controllers {

class Manager{
    public:Manager(){}

        void update(Robot_info &robot,Gait_info &gait,Swing_info &swing);

        void motor_cmd(const Robot_info& robot, ros::Publisher& pub);

    private:
        double kp_cmd = 60;
        double kd_cmd = 0.7;
        /**
         * @brief 摆动相位置控制
         * @param 获取参数 计算Robot_info的最终电机控制参数
         */
        void swing_pos_control(Robot_info &robot,Gait_info &gait,Swing_info &swing);

        /**
         * @brief 摆动相力矩控制（）
         * @param 获取参数 计算Robot_info的最终电机控制参数
         */
        void swing_torque_control(Robot_info &robot,Gait_info &gait,Swing_info &swing);

        /**
         * @brief 支撑相位置控制
         * @param 获取参数 计算Robot_info的最终电机控制参数
         */
        void stand_pos_control(Robot_info &robot,Gait_info &gait);

        /**
         * @brief 阻尼控制
         * @param 
         */
        void damping_control(Robot_info &robot);

};
}
#endif 