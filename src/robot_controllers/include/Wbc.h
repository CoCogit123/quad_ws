#ifndef WBC_H
#define WBC_H

#include "Common.h"
#include "Utils.h"
#include <ros/ros.h>

//quadprog++
#include "QuadProg++/Array.hh" 
#include "QuadProg++/QuadProg++.hh" 

namespace controllers {

    /**
     * @brief WBC
     * @details 
     */
    class Wbc{
    public:
        /**
         * @brief 构造函数
         */
        Wbc(Robot_info &robot){
            //提前分配内存 按照最大的分配
            J_pre[0].resize(12,18); J_pre[1].resize(3,18); J_pre[2].resize(3,18); J_pre[3].resize(12,18);
            J[0].resize(12,18); J[1].resize(3,18); J[2].resize(3,18); J[3].resize(12,18);
            Jdtqdt[0].resize(12); Jdtqdt[1].resize(3); Jdtqdt[2].resize(3); Jdtqdt[3].resize(12);
            x_delta_cmd[0].resize(12); x_delta_cmd[1].resize(3); x_delta_cmd[2].resize(3); x_delta_cmd[3].resize(12);
            x_dt_cmd[0].resize(12); x_dt_cmd[1].resize(3); x_dt_cmd[2].resize(3); x_dt_cmd[3].resize(12);
            x_ddt_cmd[0].resize(12); x_ddt_cmd[1].resize(3); x_ddt_cmd[2].resize(3); x_ddt_cmd[3].resize(12);
            for(int i=0;i<4;i++)
            {
                J_pre[i].setZero(); J[i].setZero(); Jdtqdt[i].setZero();
                x_delta_cmd[i].setZero(); x_dt_cmd[i].setZero(); x_ddt_cmd[i].setZero();
            }

            qp_G.resize(18,18);  qp_g0.resize(18); qp_g0.setZero();
            dyn_CE.resize(6,18);  qp_CE.resize(18,6);  qp_ce.resize(6);
            dyn_CI.resize(40,18);  qp_CI.resize(18,40);  qp_ci.resize(40);
            //初始化权重
            for(int i=0;i<6;i++)
            {
                qp_G(i,i) = 2*weight_q1;
                qp_G(6+i,6+i) = 2*weight_q2;
                qp_G(12+i,12+i) = 2*weight_q2;
            }

            //全部腿的约束
            CA.resize(5*4,3*4);
            UB.resize(5*4);
            LB.resize(5*4);

            //单条腿摩擦锥约束
            ci.resize(5,3);
            ci.setZero();
            ci(0,0) = -1; ci(0,2) = robot.mu; //第一行
            ci(1,0) =  1; ci(1,2) = robot.mu; //第二行
            ci(2,1) = -1; ci(2,2) = robot.mu; //第三行
            ci(3,1) =  1; ci(3,2) = robot.mu; //第四行
            ci(4,2) = 1; //第五行

            ubi.resize(5);
            lbi.resize(5);
            ubi << 1e10, 1e10, 1e10, 1e10, 150;
            lbi.setZero();

            //最终融合后的
            end_q_ddt.resize(18);
            end_f.resize(12);
            end_torque.resize(18);

            //pid 
            //转动
            ori_kp.diagonal() = Eigen::Vector3d(100, 100, 100);
            ori_kd.diagonal() = Eigen::Vector3d(10, 10, 10);
            //平动
            trans_kp.diagonal() = Eigen::Vector3d(100, 100, 100);
            trans_kd.diagonal() = Eigen::Vector3d(8, 8, 8);
            //摆动相
            swing_kp.diagonal() = Eigen::Vector3d(100, 100, 100);
            swing_kd.diagonal() = Eigen::Vector3d(5, 5, 5);

        }
        /**
         * @brief 核心更新函数 nullspace求解 暴力迭代
         * 
         */
        void kin_wbc(Robot_info &robot,Gait_info &gait,Swing_info &swing,double dt);

        /**
         * @brief 核心更新函数 浮动基动力学部分 并得到结果
         * 
         */
        void wbic(Robot_info &robot,Gait_info &gait,Wbc_info &wbc);

        void debug(Wbc_info &wbc);

        
    private:
        // ----------------
        // kin_wbc
        // J_pre 和 J Jdtqdt等除了目标量都分配了可能的最大维度 再去resize就不会触发重新进行内存分配
        // ----------------
        //目标量 
        Vector18d q_delta_cmd[4];
        Vector18d q_dt_cmd[4];
        Vector18d q_ddt_cmd[4];
        //中间变量
        Matrix18d N_pre; //这是共用的
        Eigen::MatrixXd J_pre[4]; //非共用
        //传入量
        Eigen::MatrixXd J[4];
        Eigen::VectorXd Jdtqdt[4];
        Eigen::VectorXd x_delta_cmd[4];
        Eigen::VectorXd x_dt_cmd[4];
        Eigen::VectorXd x_ddt_cmd[4];

        // ----------------
        // wbic
        // ----------------
        Eigen::MatrixXd qp_G;
        Eigen::VectorXd qp_g0;
        Eigen::MatrixXd dyn_CE;
        Eigen::MatrixXd qp_CE; //dyn_CE的转置
        Eigen::VectorXd qp_ce;
        Eigen::MatrixXd dyn_CI;
        Eigen::MatrixXd qp_CI; //dyn_CI的转置
        Eigen::VectorXd qp_ci;
        //全部腿的约束
        Eigen::MatrixXd CA;
        Eigen::VectorXd UB;
        Eigen::VectorXd LB;
        //摩擦锥约束 一条腿的
        Eigen::MatrixXd ci;
        Eigen::VectorXd ubi;
        Eigen::VectorXd lbi;

        /******quadprog++的*******/
        //最小化
        quadprogpp::Vector<double> quadprog_x;
        quadprogpp::Matrix<double> quadprog_G;
        quadprogpp::Vector<double> quadprog_g0;
        //等式约束
        quadprogpp::Matrix<double> quadprog_CE;
        quadprogpp::Vector<double> quadprog_ce0;
        //不等式约束
        quadprogpp::Matrix<double> quadprog_CI;
        quadprogpp::Vector<double> quadprog_ci0;
        //结果
        Eigen::VectorXd qp_result;

        Eigen::VectorXd end_q_ddt; //最终融合后的
        Eigen::VectorXd end_f;
        Eigen::VectorXd end_torque;

        //控制参数 nullspace 部分
        Eigen::DiagonalMatrix<double,3> ori_kp;
        Eigen::DiagonalMatrix<double,3> ori_kd;
        Eigen::DiagonalMatrix<double,3> trans_kp;
        Eigen::DiagonalMatrix<double,3> trans_kd;
        Eigen::DiagonalMatrix<double,3> swing_kp;
        Eigen::DiagonalMatrix<double,3> swing_kd;

        double weight_q1 = 1;//运动学的权重
        double weight_q2 = 0.005;//动力学（力矩）的权重

        // ================= 1. 模板化：Eigen 任意矩阵表达式 → QuadProg++ Matrix =================
        // 支持 MatrixXd、MatrixXd::block()、MatrixXd::topLeftCorner() 等所有 Eigen 矩阵表达式
        template <typename Derived>
        void eigenToQuadProgMat(const Eigen::MatrixBase<Derived>& eigen_mat, quadprogpp::Matrix<double>& quad_mat) {
            const int rows = eigen_mat.rows();
            const int cols = eigen_mat.cols();
            
            quad_mat.resize(rows, cols);
            if (rows == 0 || cols == 0) return;

            // 映射 QuadProg++ 连续内存为行优先矩阵
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
                quad_map(&quad_mat[0][0], rows, cols);

            // 直接赋值：Eigen 自动处理表达式求值 + 存储顺序转换
            quad_map = eigen_mat;
        }

        // ================= 2. 模板化：Eigen 任意向量表达式 → QuadProg++ Vector =================
        // 支持 VectorXd、VectorXd::segment()、VectorXd::head() 等所有 Eigen 向量表达式
        template <typename Derived>
        void eigenToQuadProgVec(const Eigen::MatrixBase<Derived>& eigen_vec, quadprogpp::Vector<double>& quad_vec) {
            const int size = eigen_vec.size();
            
            quad_vec.resize(size);
            if (size == 0) return;

            // 映射 QuadProg++ 向量内存为 Eigen 向量
            Eigen::Map<Eigen::VectorXd> quad_map(&quad_vec[0], size);

            // 直接赋值：Eigen 自动处理表达式求值
            quad_map = eigen_vec;
        }

        // ================= 3. QuadProg++ Vector → Eigen 任意向量表达式  =================

        void quadProgToEigenVec(const quadprogpp::Vector<double>& quad_vec, Eigen::VectorXd& eigen_vec) {
            const int size = quad_vec.size();
            
            // 调整 Eigen 向量大小
            eigen_vec.resize(size);
            if (size == 0) return;

            // 关键：直接映射 QuadProg++ 向量的连续内存
            // &quad_vec[0] 是连续内存的起始地址（源码中 Vector::v 的位置）
            Eigen::Map<const Eigen::VectorXd> quad_map(&quad_vec[0], size);

            // 赋值：Eigen 自动处理（无存储顺序问题，向量是一维的）
            eigen_vec = quad_map;
        }


    };

}

#endif