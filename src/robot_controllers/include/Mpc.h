#ifndef MPC_H
#define MPC_H

#include "Common.h"
#include "Gait.h"
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>

#define PLAN_HORIZON 10

namespace controllers {
    /**
     * @brief mpc
     * @details 主要就是得到 qp问题的 H g C c_up c_low； 从而更新流程就是 1.计算期望轨迹 2.计算J的参数 A_qp B_qp  Q R  3.计算约束参数 C c_up c_low
     */
    class Mpc{
        public:
            /**
             * @brief 构造函数
             */
            Mpc(){}

            /**
             * @brief 初始化函数
             */
            void init(Robot_info &robot_info);

            /**
             * @brief 核心更新函数 (每控制周期调用一次)
             */
            void update(Robot_info &robot_info,Gait_info& gait_info,double dt);

        private:
            Vector13d x_0;//当前状态 1维度
            Eigen::Matrix<double, 13 * PLAN_HORIZON, 1> X_d;//目标状态变量 

            Matrix13x13d A_k;//离散后的状态矩阵
            Matrix13x12d B_k;
            Eigen::Matrix<double, 13 * PLAN_HORIZON, 13> A_qp; //预测N个周期的预测矩阵
            Eigen::Matrix<double, 13 * PLAN_HORIZON, 12 * PLAN_HORIZON> B_qp;

            Eigen::Matrix<double, 13 * PLAN_HORIZON, 1> q_weights; //构造Q、R矩阵辅助用  
            Eigen::Matrix<double, 12 * PLAN_HORIZON, 1> r_weights;
            Eigen::DiagonalMatrix<double, 13 * PLAN_HORIZON> Q; //完整的Q、R矩阵 （对角形式）
            Eigen::DiagonalMatrix<double, 12 * PLAN_HORIZON> R;

            //J代价函数
            Eigen::Matrix<double, 12 * PLAN_HORIZON, 12 * PLAN_HORIZON> dense_hessian;
            Eigen::SparseMatrix<double> hessian;
            Eigen::Matrix<double, 12 * PLAN_HORIZON, 1> gradient;
            //约束
            Eigen::Matrix<double, 20 * PLAN_HORIZON, 1> lb;//约束上下
            Eigen::Matrix<double, 20 * PLAN_HORIZON, 1> ub;
            Eigen::SparseMatrix<double> C; // 约束矩阵C

    };


}

#endif