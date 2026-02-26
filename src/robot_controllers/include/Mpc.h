#ifndef MPC_H
#define MPC_H

#include "Common.h"
#include "Gait.h"
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>

#define use_solver 1  // 0:osqp-eigen  1:qpoases

#if use_solver == 1   //qpoases
    #include <qpOASES.hpp>
    // =================== 类型定义 (关键优化) ===================
    // 强制使用 RowMajor (行优先)，与 qpOASES 内存布局一致，实现 .data() 指针直传
    //矩阵
    template <int R, int C> using RowMat = Eigen::Matrix<double, R, C, Eigen::RowMajor>;
    //向量（列向量） 向量没区别 还是正常模式进行计算 
    template <int N> using ColVec = Eigen::Matrix<double, N, 1>;
#elif use_solver == 0  //osqp-eigen
    #include "OsqpEigen/OsqpEigen.h"
    //矩阵 默认使用列优先
    template <int R, int C> using RowMat = Eigen::Matrix<double, R, C>;
    //向量（列向量） 向量没区别 还是正常模式进行计算 
    template <int N> using ColVec = Eigen::Matrix<double, N, 1>;
#endif

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
        #if use_solver == 0 //osqp-eigen
            Mpc()
        #elif use_solver == 1 //qpoases
            Mpc() : qp_solver(12 * HORIZON, 20 * HORIZON) // qpOASES 初始化矩阵
        #endif
            {

                X_des.setZero();
                B_dt.setZero();
                A_qp.setZero();
                B_qp.setZero();
                hessian.setZero();
                gradient.setZero();
                C.setZero();
                x_now.setZero();

            }
            ~Mpc(){}

            // 初始化权重和求解器
            void init(Robot_info& robot, ColVec<13> &q_weights_, ColVec<12> &r_weights_);

            // 核心计算函数
            void update(Robot_info& robot,Gait_info& gait,double dt);

            bool mpc_init_flag = false;
            ColVec<12 * HORIZON> qp_solution;

            ColVec<13> x_now;//当前状态（1周期）
            ColVec<13 * HORIZON> X_des;//期望状态（HORIZON周期）

        private:
            double f_max = 150.0;
            double protect_degree = 1.0;//递推期望轨迹的保护堵转系数 越大越想改变状态

            // 动力学矩
            RowMat<13,13> A_dt; // 离散 A 一个周期内
            RowMat<13,12> B_dt; // 离散 B

            // 预测大矩阵
            // A_qp: [A; A^2; ... A^N]
            RowMat<13 * HORIZON, 13> A_qp; 
            // B_qp: 块下三角矩阵
            RowMat<13 * HORIZON, 12 * HORIZON> B_qp; 

            // QP 标准形式: min 1/2 x'Hx + g'x   s.t. lb <= Cx <= ub
            RowMat<12 * HORIZON, 12 * HORIZON> hessian; // Hessian
            ColVec<12 * HORIZON> gradient;                            // Gradient
            RowMat<20 * HORIZON, 12 * HORIZON> C; // Constraint Matrix
            ColVec<20 * HORIZON> lba, uba;                     // 约束边界lba uba

            // 权重 DiagonalMatrix在内存中只存储对角线上的那一个向量 不用优化先行问题
            Eigen::DiagonalMatrix<double, 13 * HORIZON> Q;
            Eigen::DiagonalMatrix<double, 12 * HORIZON> R;
      
            #if use_solver == 0 //osqp-eigen
                //需要稀疏化的矩阵
                Eigen::SparseMatrix<double> C_sparse;
                Eigen::SparseMatrix<double> hessian_sparse;
                //求解器
                OsqpEigen::Solver osqp_solver;
            #elif use_solver == 1 //qpoases
                // qpOASES 实例
                qpOASES::QProblem qp_solver;
                qpOASES::Options options;
            #endif
            
            
    };

}

#endif