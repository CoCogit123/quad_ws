#ifndef SWING_H
#define SWING_H

#include "Common.h"
#include "Utils.h"

namespace controllers{

class Swing{
    public:
        Swing(){
            k_foot_ = 0.015;
            last_gait_type_.setZero();
        }
        /**
         * @brief 核心更新函数 (每控制周期调用一次)
         * @param gait_data 引用外部的 Gait_info 和 Gait_info数据总线，函数会更新 Swing_info
         */
        void update(Swing_info &swing,Robot_info &robot,Gait_info &gait)
        {
            /********************************3个点********************************/ 
            for(int i=0;i<4;i++)
            {   
                //确认方向
                int dir_x,dir_y;
                switch(i)
                {
                    case 0: dir_x = 1 ; dir_y = -1; //右上
                            break;
                    case 1: dir_x = 1 ; dir_y = 1; //左上
                            break;
                    case 2: dir_x = -1 ; dir_y = -1; //右下
                            break;
                    case 3: dir_x = -1 ; dir_y = 1; //左下 
                            break;
                }
                /*****起点****/ 
                //捕捉变换帧
                if(last_gait_type_[i]==1&&gait.Gait_state[i]==0)
                {
                    swing.world_POS_start_touch.col(i) = robot.world_POS.col(i);
                }
                last_gait_type_[i] = gait.Gait_state[i];
                /*****中间对称点****/
                Pos_com_touch_ = robot.world_Pos_com + robot.world_Vel_des*(1-gait.Time_swing_degree[i])*gait.time_swing;
                yaw_touch_ = robot.euler[2] + robot.body_omega_des[2]*(1-gait.Time_swing_degree[i])*gait.time_swing;
                //机体系
                body_POS_mid_(0,i) = dir_x*robot.hx + robot.p_x_offest;
                body_POS_mid_(1,i) = dir_y*(robot.hy + robot.p_y_offest + robot.l1 );
                body_POS_mid_(2,i) = 0;
                //世界系
                Vector3d euler1_;
                euler1_ << 0, 0, yaw_touch_;
                swing.world_POS_mid_touch.col(i) = Pos_com_touch_ + utils::euler_to_rot(euler1_)*body_POS_mid_.col(i);
                /*****终点****/
                //增量
                delta_P1_.col(i) = robot.world_Vel_des*gait.time_stand/2;  
                Vector3d euler2_;
                euler2_ << 0, 0, robot.body_omega_des[2]*gait.time_stand/2;
                delta_P2_.col(i)= utils::euler_to_rot(euler1_)*( utils::euler_to_rot(euler2_)*body_POS_mid_.col(i) - body_POS_mid_.col(i));
                delta_P3_.col(i) = k_foot_*( robot.world_Vel_com - robot.world_Vel_des);
                delta_P4_.col(i) = (robot.world_Pos_com[2]/9.8)*utils::skew(robot.world_Vel_com) * robot.body_Omega;
                swing.world_POS_end_touch.col(i) = swing.world_POS_mid_touch.col(i) + delta_P1_.col(i) + delta_P2_.col(i) + delta_P3_.col(i) + delta_P4_.col(i);
                /********************************贝塞尔跟踪点********************************/
                Vector3d bezier_start,bezier_end;
                bezier_start =  robot.body_Rot_world.transpose()*(swing.world_POS_start_touch.col(i) - robot.world_Pos_com);
                bezier_start[0] -= dir_x*robot.hx;
                bezier_start[1] -= dir_x*robot.hy;
                bezier_end = robot.body_Rot_world.transpose()*(swing.world_POS_end_touch.col(i) - robot.world_Pos_com);
                bezier_end[0] -= dir_x*robot.hx;
                bezier_end[1] -= dir_x*robot.hy;
                if(gait.Gait_state[i]==0)
                {
                   swing.link1_POS_foot.col(i) = bezier_pos(bezier_start,bezier_end,swing.swing_high,gait.Time_swing_degree[i]);
                   swing.link1_VEL_foot.col(i) = bezier_vel(bezier_start,bezier_end,swing.swing_high,gait.Time_swing_degree[i],gait.time_swing);
                }
            }
        }
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

        Vector3d bezier_pos(Vector3d start,Vector3d end,double swing_high,double devel)
        {
            // 1. 钳位保护
            if (devel < 0.0) devel = 0.0;
            if (devel > 1.0) devel = 1.0;

            Vector3d result;

            // 2. XY轴: 3阶贝塞尔 (S形曲线: 3t^2 - 2t^3)
            // 相当于 start + (end-start) * S(t)
            double t2 = devel * devel;
            double t3 = t2 * devel;
            double s_xy = 3 * t2 - 2 * t3;

            result.x() = start.x() + (end.x() - start.x()) * s_xy;
            result.y() = start.y() + (end.y() - start.y()) * s_xy;

            // 3. Z轴: 分段贝塞尔 (提腿 -> 落腿)
            // 修正：确保中间点比起点和终点都高
            double z_max = std::max(start.z(), end.z()) + swing_high;

            if (devel < 0.5) {
                // 上升段 (0 ~ 0.5 -> 映射到 0 ~ 1)
                double t = devel * 2.0;
                double t_sq = t * t;
                double t_cu = t_sq * t;
                double s_z = 3 * t_sq - 2 * t_cu;
                
                result.z() = start.z() + (z_max - start.z()) * s_z;
            } else {
                // 下降段 (0.5 ~ 1.0 -> 映射到 0 ~ 1)
                double t = (devel - 0.5) * 2.0;
                double t_sq = t * t;
                double t_cu = t_sq * t;
                double s_z = 3 * t_sq - 2 * t_cu;
                
                result.z() = z_max + (end.z() - z_max) * s_z;
            }

            return result;
        }
        
        Vector3d bezier_vel(Vector3d start, Vector3d end, double swing_high, double devel, double swing_time)
        {
            // 1. 安全检查
            if (devel < 0.0) devel = 0.0;
            if (devel > 1.0) devel = 1.0;
            if (swing_time < 1e-4) swing_time = 1e-4; // 防止除以零

            Vector3d result;

            // 2. XY轴导数 (对相位)
            // d(3t^2 - 2t^3)/dt = 6t - 6t^2 = 6t(1-t)
            double s_xy_dot = 6.0 * devel * (1.0 - devel);
            
            result.x() = (end.x() - start.x()) * s_xy_dot;
            result.y() = (end.y() - start.y()) * s_xy_dot;

            // 3. Z轴导数 (对相位)
            // 修正：确保中间点比起点和终点都高
            double z_max = std::max(start.z(), end.z()) + swing_high;

            if (devel < 0.5) {
                // 上升段: t = 2*phase
                double t = devel * 2.0;
                double s_z_dot = 6.0 * t * (1.0 - t);
                
                // 链式法则：内部系数 2.0
                result.z() = (z_max - start.z()) * s_z_dot * 2.0;
            } else {
                // 下降段: t = 2*(phase - 0.5)
                double t = (devel - 0.5) * 2.0;
                double s_z_dot = 6.0 * t * (1.0 - t);
                
                // 链式法则：内部系数 2.0
                result.z() = (end.z() - z_max) * s_z_dot * 2.0;
            }

            // 4. 核心转换：相位导数 -> 时间导数
            // v(m/s) = v(phase) / swing_time
            return result / swing_time;
        }
};




}

#endif