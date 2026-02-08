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
        Gait(){}
        /**
         * @brief 核心更新函数 (每控制周期调用一次)
         * @param gait_data 引用外部的 Gait_info 数据总线，函数会更新其中的状态变量
         * @param time 控制器运行时长
         */
        void update(Gait_info& gait_data,double time);
        
    private:
        /**
         * @brief 根据修改模式对固定参数进行修改
         */
        void change_mode(Gait_info& gait_data);
        
        double start_time_ = 0;//从切换模式起的计时
    };
}

#endif