#ifndef DEBUG_H
#define DEBUG_H

// **************************
// 自定义头文件 
// **************************
#include "Common.h"
#include "Gait.h"
#include "Swing.h"
//自定义消息包
#include <custom_msgs/Motor_state.h>
#include <custom_msgs/Motor_control.h>
#include <custom_msgs/Sim_info.h>


namespace controllers {
    /**
     * @brief 封装机器人信息打印调试函数
     * @param robot  Robot_info结构体
     * @param freq   打印频率 (Hz)
     */
    void Debug_robot_info(const Robot_info& robot, double freq);

    /**
     * @brief 打印步态逻辑相关信息
     * @param gait  Gait_info结构体
     * @param freq  打印频率 (Hz)
     */
    void Debug_gait_info(const Gait_info& gait, double freq);

    /**
     * @brief 打印摆动腿相关信息 (落足点、轨迹跟踪、摆动高度)
     * @param swing  Swing_info结构体
     * @param freq   打印频率 (Hz)
     */
    void Debug_swing_info(const Swing_info& swing, double freq);


    }
#endif