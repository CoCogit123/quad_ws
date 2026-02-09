#ifndef SWING_H
#define SWING_H

#include "Common.h"
#include "Utils.h"

namespace controllers{

class Swing{
    public:
        Swing(){
            k_foot_ = 0.015;
            last_gait_type_.setConstant(1);
        }
        /**
         * @brief 核心更新函数 (每控制周期调用一次)
         * @param gait_data 引用外部的 Gait_info 和 Gait_info数据总线，函数会更新 Swing_info
         */
        void update(Swing_info &swing,Robot_info &robot,Gait_info &gait);

    private:
        Vector3d Pos_com_touch_; //估计的落足时质心位置（世界系）
        double yaw_touch_; //估计的落足时yaw
        Matrix3x4d body_POS_mid_; //四条腿的对称点机体系
        Matrix3x4d delta_P1_; //平动项增量
        Matrix3x4d delta_P2_; //转动项增量
        Matrix3x4d delta_P3_; //速度跟踪项增量
        Matrix3x4d delta_P4_; //匀速圆周速度项增量
        double k_foot_;
        Vector4i last_gait_type_;

        Vector3d bezier_pos(Vector3d start,Vector3d end,double swing_high,double devel);
        
        Vector3d bezier_vel(Vector3d start, Vector3d end, double swing_high, double devel, double swing_time);
    
};




}

#endif