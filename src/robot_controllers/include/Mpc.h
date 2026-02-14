#ifndef MPC_H
#define MPC_H

#include "Common.h"
#include "Gait.h"
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>

//qpoases
#include <qpOASES.hpp>

// =================== 类型定义 (关键优化) ===================
// 强制使用 RowMajor (行优先)，与 qpOASES 内存布局一致，实现 .data() 指针直传
//矩阵
using Qptype = qpOASES::real_t;
template <int R, int C> using RowMat = Eigen::Matrix<Qptype, R, C, Eigen::RowMajor>;
using MatrixXd = Eigen::Matrix<Qptype, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
//向量（列向量） 向量没区别 还是正常模式进行计算 
template <int N> using ColVec = Eigen::Matrix<Qptype, N, 1>;
using VectorXd = Eigen::Matrix<Qptype, Eigen::Dynamic, 1>;
// =================== 配置参数 ===================
constexpr int HORIZON = 10;      // 预测步长 (MIT通常用10)

namespace controllers {
    /**
     * @brief mpc
     * @details 主要就是得到 qp问题的 H g C c_up c_low； 从而更新流程就是 1.计算期望轨迹 2.计算J的参数 A_qp B_qp  Q R  3.计算约束参数 C c_up c_low
     */
    class Mpc
    {
        public:
            Mpc()//只初始化矩阵
            {
                // === 2. Eigen 内存预分配 ===
                A_qp.resize(13 * HORIZON, 13);
                B_qp.resize(13 * HORIZON, 12 * HORIZON);
                hessian.resize(12 * HORIZON, 12 * HORIZON);
                C.resize(HORIZON, 12 * HORIZON);
                X_des.resize(13 * HORIZON);
                gradient.resize(12 * HORIZON);
                lb.resize(20 * HORIZON);
                ub.resize(20 * HORIZON);

                X_des.setZero();
                B_dt.setZero();
                A_qp.setZero();
                B_qp.setZero();
                hessian.setZero();
                gradient.setZero();
                C.setZero();
                x_now.setZero();
            }
            ~Mpc();

            // 初始化权重和求解器
            void init(Robot_info& robot, VectorXd &q_weights_, VectorXd &r_weights_);

            // 核心计算函数
            void update(Robot_info& robot,Gait_info& gait,double dt);

        private:
            double f_max = 150;

            // 动力学矩阵
            ColVec<13> x_now;//当前状态（1周期）
            VectorXd X_des;//期望状态（HORIZON周期）
            RowMat<13,13> A_dt; // 离散 A 一个周期内
            RowMat<13,12> B_dt; // 离散 B

            // 预测大矩阵
            // A_qp: [A; A^2; ... A^N]
            MatrixXd A_qp; 
            // B_qp: 块下三角矩阵
            MatrixXd B_qp; 

            // QP 标准形式: min 1/2 x'Hx + g'x   s.t. lb <= Cx <= ub
            MatrixXd hessian; // Hessian
            VectorXd gradient;                            // Gradient
            MatrixXd C; // Constraint Matrix
            VectorXd lb, ub;                     // 约束边界

            // 权重 DiagonalMatrix在内存中只存储对角线上的那一个向量 不用优化先行问题
            Eigen::DiagonalMatrix<double, 13 * HORIZON> Q;
            Eigen::DiagonalMatrix<double, 12 * HORIZON> R;

            // qpOASES 实例
            qpOASES::QProblem qp_solver;
            qpOASES::Options options;
            bool first_run = true;
            Eigen::VectorXd qp_solution;
    };


}

#endif