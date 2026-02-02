#include "Estimate.h"
#include "Utils.h"
namespace controllers{

void Estimate::init(Robot_info &robot_data)
{
    double Z_start = 0.6;
    //初始化 P_k0_update 以及 X_k0_update
    P_k0_update.setIdentity();
    P_k0_update = P_k0_update * 3;
    //X 初始化基础高度  （地面为0）
    X_k0_update.setZero();
    X_k0_update.segment<3>(0) =  Eigen::Vector3d(0, 0, Z_start);
    //足端位置
    for (int i = 0; i < 4; ++i) {
        X_k0_update.segment<3>(6 + i * 3) = robot_data.motor_q.segment<3>(i * 3); //世界坐标系下(一开始其实也是机体坐标系)
    }

    //A、B、H矩阵
    A.setIdentity();
    B.setZero();
    eye3.setIdentity(); // I（3*3）
    H.setZero();
    for (int i=0; i<4; ++i) {
        H.block<3,3>(i*3,0) = -eye3;  //  足端相对于机体的位置
        H.block<3,3>(i*3,6+i*3) = eye3;  //foot pos   和上面组合 变成 p_foot-p_body
        H.block<3,3>(12+i*3,3) = eye3;  // vel 
        H(24+i,8+i*3) = 1;  // height z of foot
    }
    //Q、R协方差矩阵
    Q.setZero();
    Q.block<3,3>(0,0) = PROCESS_NOISE_POS*eye3;               // position transition
    Q.block<3,3>(3,3) = PROCESS_NOISE_VEL*eye3;               // velocity transition
    for (int i=0; i<4; ++i) {
        Q.block<3,3>(6+i*3,6+i*3) = PROCESS_NOISE_FOOT*eye3;  // foot position transition
    }

    R.setZero();
    for (int i=0; i<4; ++i) {
        R.block<3,3>(i*3,i*3) = SENSOR_NOISE_P*eye3;                        // fk estimation
        R.block<3,3>(12+i*3,12+i*3) = SENSOR_NOISE_V*eye3;      // vel estimation
        R(24+i,24+i) = SENSOR_NOISE_Z;                               // height z estimation
    }
}


//全是机体的
// Eigen::Matrix3d root_rot_mat,Eigen::Vector3d root_lin_acc_body,Eigen::Vector3d root_omega,
//                                     Eigen::Matrix<double, 3, 4> foot_pos_body,Eigen::Matrix<double, 3, 4> foot_vel_body,
//                                     int contacts[4],double dt

void Estimate::update(Robot_info &robot_data,double dt)
{
    //更新 A，B
    A.block<3, 3>(0, 3) = dt * eye3;
    B.block<3, 3>(3, 0) = dt * eye3;
    //更新U = a 输入为加速度 (世界坐标系)
    U_k0_update = robot_data.rot_mat*robot_data.imu_acc + Eigen::Vector3d(0, 0, -9.8);
    //根据支撑相的进度来决定协方差矩阵Q、R的大小，使得数据相对稳定  另一种是根据力 
    for(int i=0;i<4;i++)
    {   
        estimated_contacts[i] =  robot_data.contact_state[i];
    }
    //更新Q R
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_POS * dt / 20.0 * eye3;
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VEL * dt * 9.8 / 20.0 * eye3;
    for(int i=0;i<4;i++)
    {
        //对于摆动相 预测协方差矩阵中对应的足端位置的项的协方差直接拉满
        Q.block<3,3>(6+i*3,6+i*3) = 
            (1 + (1 - estimated_contacts[i]) * 1e3) * dt * PROCESS_NOISE_FOOT * eye3;
        //对于摆动相 测量协方差矩阵中对应uresidual （足端位置和质心位置之差）的项的协方差拉满
        R.block<3,3>(i*3,i*3) = 
            (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_P * eye3;
        //对于摆动相，测量协方差矩阵中对应足端速度的项的协方差拉满。
        R.block<3,3>(12+i*3,12+i*3) = 
            (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_V * eye3;
        //在平地上走的前提下。如果足端没有触地，测量协方差矩阵中对应足端高度的项的协方差拉满。
        R(24+i,24+i) = 
            (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_Z;
    }
    //预测
    X_k1_est = A * X_k0_update + B * U_k0_update;
    P_k1_est = A * P_k0_update * A.transpose() + Q; 
   

    Z_k1_est = H * X_k1_est;
    
    //观测值
    for (int i = 0; i < 4; i++)
    {
        //足端位置——机体坐标系
        Eigen::Vector3d fk_pos = robot_data.foot_pos_body[i]; 
        //加入旋转 足端与质心位置差（世界坐标系）
        Z_k0_update.block<3,1>(i*3,0) = robot_data.rot_mat*fk_pos;
        // 足端推测出来的质心速度 足端在全局坐标系下的速度 足端速度_world=Rot*（vel_foot_body+w[x]*pos_foot_body）+ v_root;
        // 支撑相下 有相对速度为0 则 足端速度_world = 0; 则  v_root = -Rot*（vel_foot_body+w[x]*pos_foot_body）;
        Eigen::Vector3d leg_v = -robot_data.foot_vel_body[i] - utils::skew( robot_data.imu_gyro )*fk_pos;
        // 支撑相的时候就用算出来的足端速度（因为足端的速度是根据足端的位置算出来的，而此时足端的位置是可靠的），摆动相的时候（足端位置和速度不可靠）用质心的速度
        Z_k0_update.block<3,1>(12+i*3,0) =
                (1.0-estimated_contacts[i])*X_k0_update.segment<3>(3) +  estimated_contacts[i]*robot_data.rot_mat*leg_v;      // vel estimation
        //! 支撑相的时候默认足端高度是0（这个假设yxy的论文中提到过）,摆动相的时候通过质心高度和编码器得到的足端位置计算（加入足端z，是因为他会对质心的运动状态有影响）
        Z_k0_update(24+i) =
                (1.0-estimated_contacts[i])*(X_k0_update(2)+fk_pos(2));     
    }

    //Hp-H + R
    S = H * P_k1_est * H.transpose() + R;  
    S = 0.5*(S+S.transpose()); //保证对称性
    // 
    error_z = Z_k0_update - Z_k1_est;
    ////!  s^-1*error_z ;
    // Serror_z = S.fullPivHouseholderQr().solve(error_z);
    Serror_z = S.llt().solve(error_z);//改进版本
    // 后验状态估计
    // x = x^-1 + p^-1*H^T* s^-1*error_z
    X_k0_update = X_k1_est + P_k1_est * H.transpose() * Serror_z; 

    //!  s^-1*H
    // SH = S.fullPivHouseholderQr().solve(H); 还有SH = S.ldlt().solve(H);
    SH = S.llt().solve(H);
    P_k0_update = P_k1_est - P_k1_est * H.transpose() * SH * P_k1_est;
    P_k0_update = 0.5 * (P_k0_update + P_k0_update.transpose());//保证对称性

    //! tricks：减少位置漂移 reduce position drift
    if (P_k0_update.block<2, 2>(0, 0).determinant() > 1e-6) {
        P_k0_update.block<2, 16>(0, 2).setZero();
        P_k0_update.block<16, 2>(2, 0).setZero();
        P_k0_update.block<2, 2>(0, 0) /= 10.0;
    }

    // root_pos =  X_k0_update.segment<3>(0);
    // root_lin_vel_world = X_k0_update.segment<3>(3);
    // root_lin_vel_body = root_rot_mat.transpose()*root_lin_vel_world;
}
}