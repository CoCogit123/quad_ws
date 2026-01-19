#ifndef GAIT_H
#define GAIT_H

#include "Common.h"

namespace controllers {
    /**
     * @brief 步态调度器类
     * @details 全局相位控制在[0,1) >1,0时-=1.0；
     *          st/sw相位则在无限接近时（0.995）令其等于1；
     */
    class Gait{
    public:
        /**
         * @brief 构造函数
         */
        Gait() { }
        /**
         * @brief 核心更新函数 (每控制周期调用一次)
         * @param gait_data 引用外部的 Gait_info 数据总线，函数会更新其中的状态变量
         * @param time 控制器运行时长
         */
        void update(Gait_info& gait_data,double time)
        {
            // =========================================================
            // Step 1: 检查是否需要切换步态
            // =========================================================
            if(gait_data.Gait_flag==1&&gait_data.time_gait_degree>=0.95)
            {
                change_mode(gait_data);
                gait_data.Gait_flag = 0;
                start_time = time;
                gait_data.time_gait_degree = 0.0;
                return;
            }
            // =========================================================
            // Step 2: 全局相位更新 (核心心跳)
            // =========================================================
            gait_data.Gait_N = (int)((time - start_time) / gait_data.time_gait);        // 第几个
            gait_data.time_gait_degree = ((time - start_time) / gait_data.time_gait)- gait_data.Gait_N; // 进度
            if(gait_data.time_gait_degree >= 1.0) { gait_data.time_gait_degree -= 1.0; }//控制在[0,1)
            // =========================================================
            // Step 3: 更新每条腿的状态 (Stance/Swing)
            // =========================================================
            for (int i = 0; i < 4; i++)
            {
                // 令偏移量归0后的进度
                double degree;
                if (gait_data.time_gait_degree >= gait_data.Gait_offset[i])
                {
                    degree = gait_data.time_gait_degree - gait_data.Gait_offset[i];
                }
                else
                {
                    degree = gait_data.time_gait_degree - gait_data.Gait_offset[i] + 1;
                }
                // 判断支撑相是否
                gait_data.Gait_state[i] = (degree < gait_data.Gait_ratio[i] ? 1 : 0);
                // 进度
                if (gait_data.Gait_state[i] == 0) // 摆动
                {
                    gait_data.Time_stand_degree[i] = 1; // 支撑进度 = 1 (完成)
                    gait_data.Time_swing_degree[i] = (degree - gait_data.Gait_ratio[i]) / (1 - gait_data.Gait_ratio[i]); // 摆动进度
                    if(gait_data.Time_swing_degree[i]>=0.995) { gait_data.Time_swing_degree[i] = 1;}
                    if(gait_data.Time_swing_degree[i]<=0.0) { gait_data.Time_swing_degree[i] = 0.0;}
                }
                else // 支撑
                {
                    gait_data.Time_swing_degree[i] = 1;                                     // 摆动进度 = 1 (完成)
                    gait_data.Time_stand_degree[i] = degree / gait_data.Gait_ratio[i]; // 支撑进度
                    if(gait_data.Time_stand_degree[i]>=0.995) { gait_data.Time_stand_degree[i] = 1;}
                    if(gait_data.Time_stand_degree[i]<=0.0) { gait_data.Time_stand_degree[i] = 0.0;}
                }
            }
        }
    private:
        /**
         * @brief 根据修改模式对固定参数进行修改
         */
        void change_mode(Gait_info& gait_data)
        {
            switch (gait_data.Gait_des) {
            //趴下
            case Gait_type::none:
                // 周期 1.0s (无意义), 占空比 1.0 (全支撑)
                gait_data.time_gait = 1.0;
                gait_data.Gait_ratio.setConstant(1.0);
                gait_data.Gait_offset.setZero();
                break;
            //stand
            case Gait_type::stand:
                // 周期 1.0s (无意义), 占空比 1.0 (全支撑)
                gait_data.time_gait = 1.0;
                gait_data.Gait_ratio.setConstant(1.0);
                gait_data.Gait_offset.setZero();
            //walk
            case Gait_type::walk:
                // 周期 0.8s, 占空比 0.75
                gait_data.time_gait = 0.8;
                gait_data.Gait_ratio.setConstant(0.75);
                // 偏移量
                gait_data.Gait_offset << 0.25, 0.75, 0.5, 0.0;
                break;
            //trot
            case Gait_type::trot:
            default:
                // 周期 0.6s, 占空比 0.75
                gait_data.time_gait = 0.6;
                gait_data.Gait_ratio.setConstant(0.5);
                gait_data.Gait_offset << 0.5, 0.0, 0.0, 0.5;
                break;
            }
        }
        double start_time = 0;//从切换模式起的计时
    };
}

#endif