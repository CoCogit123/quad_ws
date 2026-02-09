#include "Estimate.h"
#include "Utils.h"
#include <ros/ros.h>
namespace controllers{

void Estimate::init(Robot_info &robot_data) 
{
    //初始化算法需要的初值P和X；
        //P
    double Z_start = 0.2;
    P.setIdentity();
    P = P * 3;
        //X 位置3 速度3 足端位置12
    X.setZero();//全0
    X[2] = Z_start;//位置
    for (int i = 0; i < 4; ++i) //足端世界下位置 
    {
        X.segment<3>(6 + i * 3) = robot_data.body_Rot_world * robot_data.body_POS.col(i) + X.segment<3>(0);
    }
    //设置算法中不变的部分 减少重复计算
        //单位阵
    eye3.setIdentity();
        //A
    A.setIdentity();
        //B
    B.setZero();
        //Q
    Q.setZero();
    Q.block<3,3>(0,0) = PROCESS_NOISE_POS*eye3;               // position transition
    Q.block<3,3>(3,3) = PROCESS_NOISE_VEL*eye3;               // velocity transition
    for (int i=0; i<4; ++i) {
        Q.block<3,3>(6+i*3,6+i*3) = PROCESS_NOISE_FOOT*eye3;  // foot position transition
    }
        //R
    R.setZero();
    for (int i=0; i<4; ++i) {
        R.block<3,3>(i*3,i*3) = SENSOR_NOISE_P*eye3;                        // fk estimation
        R.block<3,3>(12+i*3,12+i*3) = SENSOR_NOISE_V*eye3;      // vel estimation
        R(24+i,24+i) = SENSOR_NOISE_Z;                               // height z estimation
    }
        //H
    H.setZero();
    for (int i=0; i<4; ++i) {
        H.block<3,3>(i*3,0) = -eye3;  //  足端相对于机体的位置
        H.block<3,3>(i*3,6+i*3) = eye3;  //foot pos   和上面组合 变成 p_foot-p_body
        H.block<3,3>(12+i*3,3) = eye3;  // vel 
        H(24+i,8+i*3) = 1;  // height z of foot
    }
}


void Estimate::update(Robot_info &robot_data,Gait_info& gait_data,double dt)
{
    //公式一: 计算先验估计值X_est
        //更新 A，B 中变化部分
    A.block<3, 3>(0, 3) = dt * eye3;
    B.block<3, 3>(3, 0) = dt * eye3;
        //更新U = a 输入为加速度 (世界坐标系)
    U = robot_data.body_Rot_world * robot_data.body_Acc + Eigen::Vector3d(0, 0, -9.81);
        //更新预测X_est
    X_est = A * X + B * U;

    //公式二：计算先验误差协方差矩阵P_est
        //更新Q和R  Q（18）：过程噪声协方差矩阵 位置3 速度3 足端位置12（3组足端位置）
        //          R（28）：测量噪声协方差矩阵 世界足端位置12 足端速度12 足端高度4
        //对摆动腿的置信度直接拉满 支撑相则根据相位情况取值/置信度（0-1） 置信度低则会放大
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_POS * dt / 20.0 * eye3;
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VEL * dt * 9.8 / 20.0 * eye3; 
    for(int i=0;i<4;i++)
    {   
        //先更新置信度
        if(gait_data.Gait_state[i] == 0)//摆动腿 拉满拉满
        {
            trust[i] = 0;
        }else if(gait_data.Gait_state[i] == 1)//支撑时 分为3种情形
        {
            if(gait_data.Gait_mode == walk || gait_data.Gait_mode == trot)
            {
                if( gait_data.Time_stand_degree[i] <= 0.2 ) { trust[i] = gait_data.Time_stand_degree[i]/0.2; }
                else if( gait_data.Time_stand_degree[i] >= 0.8 ) { trust[i] = (1-gait_data.Time_stand_degree[i])/0.2; }
                else { trust[i] = 1; }
            }else 
            {
                trust[i] = 1;
            }
        }
        //足端位置
        Q.block<3,3>(6+i*3,6+i*3) = (1 + (1 - trust[i]) * 1e3) * dt * PROCESS_NOISE_FOOT * eye3;
        // 旋转矩阵*机体足端位置 + 世界位置 -> 世界足端位置  --》 -旋转矩阵*机体足端位置 = 世界位置 - 世界足端位置
        R.block<3,3>(i*3,i*3) = (1 + (1 - trust[i]) * 1e3) * SENSOR_NOISE_P * eye3;
        // 足端世界速度 -》 机体速度
        R.block<3,3>(12+i*3,12+i*3) = (1 + (1 - trust[i]) * 1e3) * SENSOR_NOISE_V * eye3;
        //足端高度 没触地拉满
        R(24+i,24+i) = (1 + (1 - trust[i]) * 1e3) * SENSOR_NOISE_Z;
    }
        //先验误差协方差矩阵
    P_est = A * P * A.transpose() + Q;

    //公式三： 计算卡尔曼增益Kk； 
        //注:为节省计算时间 不直接求逆 需要求逆的部分使用S 之后融合逆的部分乘以后面部分 使用llt分解加速求解 所以只计算S
    S = H * P_est * H.transpose() + R; 
    S = 0.5*(S+S.transpose()); //保证对称性
    Eigen::LLT<Matrix28d> dec(S);
    if(dec.info()!=Eigen::Success) {
        // 回退/正则化
        ROS_ERROR("estimate^ S did not llt");
    }

    
    //公式四： 根据增益计算后验状态变量
        //先导入观测值
    for (int i = 0; i < 4; i++)
    {
        Vector3d trans_pos = robot_data.body_Rot_world * robot_data.body_POS.col(i);
        Z.block<3,1>(i*3,0) =  trans_pos;  //位置
        Z.block<3,1>(12+i*3,0) = - robot_data.body_Rot_world * ( utils::skew(robot_data.body_Omega) * robot_data.body_POS.col(i) 
                                                                + robot_data.body_VEL.col(i) );//速度
        Z(24+i) = (1.0-trust[i])*( X(2) + trans_pos(2) );//Z 支撑就认为是0（着地） 摆动就按上一个时刻的估计足端世界高度来算                                                      
    }
        //观测误差 以及Kk分母乘以error_z
    error_z = Z - H * X_est;
    Serror_z = dec.solve(error_z);
        //得到后验估计 X_est + Kk分子*Serror_z
    X = X_est + P_est * H.transpose() * Serror_z;

    //公式五： 更新新的后验误差协方差矩阵
        //SH kk分母乘以H
    SH = dec.solve(H);
        //后验（单位阵 - kk分子*SH）*先验P
    P  =  P_est - P_est * H.transpose() * SH  * P_est;
    P = 0.5 * (P + P.transpose());//保证对称性


    robot_data.X_est = X;
}

}