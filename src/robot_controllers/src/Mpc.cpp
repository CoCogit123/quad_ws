#include "Mpc.h"
#include "Dynamics.h"
#include <ros/ros.h>
namespace controllers {

void Mpc::init(Robot_info& robot,ColVec<13> &q_weights_, ColVec<12> &r_weights_)
{
    // =========================================================
    // 离散的状态方程
    // =========================================================
    A_dt.setIdentity();

    // =========================================================
    // 约束方程
    // 约束形式：
    // 0: -fx + mu*fz >= 0  (即 fx <= mu*fz)
    // 1:  fx + mu*fz >= 0  (即 fx >= -mu*fz)
    // 2: -fy + mu*fz >= 0  (即 fy <= mu*fz)
    // 3:  fy + mu*fz >= 0  (即 fy >= -mu*fz)
    // 4:  fz >= 0 且 fz <= f_max
    // =========================================================
    
    //下限
    lba.setZero();
    //上限 先全部设置成无穷大 再把对应的项设置成fmax
    uba.setConstant(1e10);
    for(int i=0;i<HORIZON;++i)
    {
        uba[i*20+4+0] = f_max; //4
        uba[i*20+4+5] = f_max; //9
        uba[i*20+4+10] = f_max; //14
        uba[i*20+4+15] = f_max; //19
    }
    // ROS_INFO_STREAM("lb矩阵内容：\n" << lb);
    // ROS_INFO_STREAM("ub矩阵内容：\n" << ub);
    //约束矩阵C
    RowMat<5, 3> C_i;
    C_i.setZero();
    C_i(0,0) = -1; C_i(0,2) = robot.mu; //第一行
    C_i(1,0) =  1; C_i(1,2) = robot.mu; //第二行
    C_i(2,1) = -1; C_i(2,2) = robot.mu; //第三行
    C_i(3,1) =  1; C_i(3,2) = robot.mu; //第四行
    C_i(4,2) = 1; //第五行
    for(int i=0;i<HORIZON;++i)
    {
        C.block<5,3>(i*20 + 0,i*12 + 0) = C_i;
        C.block<5,3>(i*20 + 5,i*12 + 3) = C_i;
        C.block<5,3>(i*20 + 10,i*12 + 6) = C_i;
        C.block<5,3>(i*20 + 15,i*12 + 9) = C_i;
    }
    // Eigen::MatrixXd C_block = C.block<20, 12>(0, 0);
    // ROS_INFO_STREAM("C矩阵内容：\n" << C_block);
    // =========================================================
    // 权重
    // =========================================================
    ColVec<13*HORIZON> Q_vec;
    ColVec<12*HORIZON> R_vec;
    for (int i = 0; i < HORIZON; ++i) {
        //segment(位置x，长度L) 提取该列向量 x位置之后的L长度， 替换为去 q_weights_ ;
        Q_vec.segment(i * 13, 13) = q_weights_;
        R_vec.segment(i * 12, 12) = r_weights_;
    }
    Q.diagonal() = Q_vec;
    R.diagonal() = R_vec;

    // =========================================================
    // 求解器初始化
    // =========================================================
    #if use_solver == 0 //osqp-eigen
        C_sparse = C.sparseView(); //一个将密集矩阵转换为稀疏矩阵视图，只存储非零元素，从而节省内存和计算资源。
    #elif use_solver == 1 //qpoases
        // === qpOASES 设置 ===
        options.setToMPC(); // 启用热启动模式
        options.printLevel = qpOASES::PL_NONE; // 关闭所有打印 (实时性关键)
        qp_solver.setOptions(options);
    #endif

}

void Mpc::update(Robot_info& robot,Gait_info& gait,double dt)
{
    // =========================================================
    // Step 0: 当前状态以及期望状态  [姿态(3) 位置(3) 角速度(3) 线速度(3) g(1)]  共13
    // =========================================================
    /************** 当前状态x_now **************/
    x_now.segment(0,3) = robot.euler;
    x_now.segment(3,3) = robot.world_Pos_com;
    x_now.segment(6,3) = robot.body_Rot_world * robot.body_Omega;
    x_now.segment(9,3) = robot.world_Vel_com;
    x_now[12] = -9.81;
    /************** 期望状态X_des 13*10**************/
    static ColVec<3> pos_des_k1 = robot.world_Pos_com;
    for (int i = 0; i < HORIZON; ++i) {
        if( i == 0 ) //k=1的期望位置
        {
            pos_des_k1 = protect_degree * pos_des_k1 + ( 1 - protect_degree ) * robot.world_Pos_com + robot.world_Vel_com * dt;
            // pos_des_k1.setZero();
            pos_des_k1[2] = robot.z_des;
            X_des.segment(3,3) = pos_des_k1; //位置
            X_des.segment(0,3).setZero();//姿态roll pitch 为0
            X_des(2) = robot.euler[2];//姿态 -- 直接为当前姿态
            X_des.segment(6,3) = robot.world_omega_des;//角速度
            X_des.segment(9,3) = robot.world_Vel_des;//线速度
            X_des[12] = -9.81;
        }else
        {
            // 计算当前步的时间偏移
            double t = (i + 1) * dt;

            X_des.segment(i*13+0,3) = robot.euler + robot.world_omega_des*t;//姿态
            X_des.segment(i*13+3,3) = pos_des_k1 + robot.world_Vel_des*t;//位置
            X_des[i*13+5] = robot.z_des;//z(保持期望高度)
            X_des.segment(i*13+6,3) = robot.world_omega_des;//角速度
            X_des.segment(i*13+9,3) = robot.world_Vel_des;//线速度
            X_des[i*13+12] = -9.81;
        }
    }

    // =========================================================
    // Step 1: 离散的状态方程
    // =========================================================
    /************** A_dt变化部分 **************/
    double cos_yaw = cos(robot.euler[2]);
    double sin_yaw = sin(robot.euler[2]);
    //旋转矩阵的转置
    Eigen::Matrix3d ang_vel_to_rpy_rate;
    ang_vel_to_rpy_rate << cos_yaw, sin_yaw, 0,
                        -sin_yaw, cos_yaw, 0,
                        0, 0, 1;
    A_dt.block<3, 3>(0, 6) = ang_vel_to_rpy_rate * dt; //euler对应行
    A_dt.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity() * dt;//pos对应行
    A_dt(11, 12) = dt;//vel(2)对应行--Z方向
    // ROS_INFO_STREAM("A矩阵内容：\n" << A_dt);
    /************** B_dt变化部分 **************/
    Eigen::Matrix<double, 3, 4> foot_pos_abs; //只考虑旋转（世界坐标系原点下的坐标）
    foot_pos_abs = robot.body_Rot_world * robot.body_POS;
    for (int i = 0; i < 4; ++i) {
        //.inverse()是逆 
        // Utils::skew(foot_pos.block<3, 1>(0, i)) 反对称矩阵 [ri]x  ri=foot_pos.block<3, 1>(0, i)；从第（0，i）取出3*1 
        B_dt.block<3, 3>(6, 3 * i) =
                robot.world_INERTIA.inverse() * utils::skew(foot_pos_abs.col(i)) * dt;
        B_dt.block<3, 3>(9, 3 * i) =
                (1 / robot.mass) * Eigen::Matrix3d::Identity() * dt;//robot.mass取11
    }
    // ROS_INFO_STREAM("B矩阵内容：\n" << B_dt);

    // =========================================================
    // Step 2: 预测状态方程
    // =========================================================
    RowMat<13, 13> A_pow; 
    A_pow.setIdentity();
    for(int i = 0; i < HORIZON; ++i) {
        // 更新 A_qp
        A_pow = A_pow * A_dt; 
        A_qp.block<13, 13>(i * 13, 0).noalias() = A_pow;

        // 更新 B_qp (块下三角)
        // 第 i 行，第 0 到 i 列
        for(int j = 0; j <= i; ++j) {
            // 优化：B_qp(i, j) = A^(i-1-j) * B
            // 利用上一行的计算结果递推： Block(i, j) = A_dt * Block(i-1, j)
            if (i == j) {
                //对角阵不变
                B_qp.block<13, 12>(i * 13, j * 12) = B_dt;
            } else {
                B_qp.block<13, 12>(i * 13, j * 12).noalias() = 
                    A_dt * B_qp.block<13, 12>((i-1) * 13, j * 12);
            }
        }
    }

    // =========================================================
    // Step 3: 约束方程 根据论文的形式 之后只判断腿的情况 修改上限的fmax  摆动腿设置为0 这样就不用怎么修改C 和ub和lb
    // =========================================================
    for(int i=0;i<HORIZON;++i)
    {
        for(int leg = 0; leg < 4; ++leg)
        {
            // 设置 fz 的上限 fz的序号
            int fz_idx = i*20 + leg*5 + 4; // 第 i 步，第 leg 条腿的第 5 个约束(fz)
            // 核心逻辑：根据步态表 gait_table 决定 f_max 摆动腿则整条腿的力为0
            if (gait.Gait_state[leg] == 1) {
                uba(fz_idx) = f_max; // 支撑相：允许出力
            } else {
                uba(fz_idx) = 0.0;   // 摆动相：强制 fz <= 0
            }
        }
    }

    // =========================================================
    // Step 4: QP问题相关矩阵
    // =========================================================
    // hessian = 2 * ( B_qp.transpose() * Q * B_qp + R );
    // gradient = 2 * B_qp.transpose() * Q * ( A_qp * x_now - X_des ); 
    //为了减少开销：
    hessian.noalias() = 2.0 * (B_qp.transpose() * Q * B_qp);
    hessian.diagonal() += 2.0 * R.diagonal();

    static ColVec<13 * HORIZON> prediction_error;
    prediction_error.noalias() = A_qp * x_now - X_des;
    prediction_error = Q * prediction_error;
    gradient.noalias() = 2.0 * (B_qp.transpose() * prediction_error);

    
    // =========================================================
    // Step 5: QP问题求解
    // =========================================================
    #if use_solver == 0 //osqp-eigen
        //稀疏化
        hessian_sparse = hessian.sparseView();
        //求解器
        if (!osqp_solver.isInitialized()) {
        osqp_solver.settings()->setVerbosity(false);
        osqp_solver.settings()->setWarmStart(true);
        osqp_solver.data()->setNumberOfVariables(12 * HORIZON);
        osqp_solver.data()->setNumberOfConstraints(20 * HORIZON); //20是约束 一条腿5个共20
        osqp_solver.data()->clearLinearConstraintsMatrix();
        osqp_solver.data()->clearHessianMatrix();
        osqp_solver.data()->setLinearConstraintsMatrix(C_sparse);
        osqp_solver.data()->setHessianMatrix(hessian_sparse);
        osqp_solver.data()->setGradient(gradient);
        osqp_solver.data()->setLowerBound(lba);
        osqp_solver.data()->setUpperBound(uba);
        osqp_solver.initSolver();
        osqp_solver.solveProblem();
        ROS_INFO("solver init success!!!!!!");
    }
    else{
        osqp_solver.updateHessianMatrix(hessian_sparse);
        osqp_solver.updateGradient(gradient);
        osqp_solver.updateLowerBound(lba);
        osqp_solver.updateUpperBound(uba);
        osqp_solver.solveProblem();
        qp_solution = osqp_solver.getSolution();
        Eigen::VectorXd qp_force = qp_solution.head(12);
        bool is_valid = !qp_force.hasNaN() && qp_force.allFinite();
        if (is_valid)
        {
            robot.mpc_force = qp_force;
        }else{
            ROS_WARN("QP solution has NaN/Inf, skip assigning mpc_force");
        }
    }
    #elif use_solver == 1 //qpoases
        qpOASES::int_t nWSR = 1000; // 最大工作集迭代次数
        qpOASES::returnValue ret1;
        qpOASES::returnValue ret2;
        // 冷启动：初始化矩阵结构
        ret1 = qp_solver.init(hessian.data(), gradient.data(), C.data(), 
                            (qpOASES::real_t*)nullptr, (qpOASES::real_t*)nullptr, 
                            lba.data(), uba.data(), 
                            nWSR);
        if(ret1 != qpOASES::SUCCESSFUL_RETURN) 
        {
            ROS_INFO("qp_solver failed ret 1= %d",ret1);
        }else
        {
            ret2 = qp_solver.getPrimalSolution(qp_solution.data());
            if(ret2 != qpOASES::SUCCESSFUL_RETURN)
            {
                ROS_INFO("qp_get result failed ret 2= %d",ret2);
            }else{
                robot.mpc_force = qp_solution.head(12);
            }
        }

    #endif



}



}