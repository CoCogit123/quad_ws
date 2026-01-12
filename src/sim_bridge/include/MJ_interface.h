#ifndef MJ_INTERFACE_H
#define MJ_INTERFACE_H
#include "mujoco/mujoco.h"
#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Dense"


class MJ_Interface {
public:
    int jointNum{0};                    //关节数量        
    std::vector<double> motor_pos;      //当前关节位置
    std::vector<double> motor_pos_Old;  //上一时刻关节位置
    std::vector<double> motor_vel;      //关节速度
    double rpy[3]{0};                   // roll,pitch and yaw of baselink
    double yaw_simgle;
    int	   yaw_N = 0;
    Eigen::Matrix3d base_rot;   // 基链接旋转矩阵
    double baseQuat[4]{0};      // 基链接四元数（调整后顺序：[x, y, z, w]）
    double basePos[3]{0};       // 基链接世界坐标位置 position of baselink, in world frame 
    double baseLinVel[3]{0};    // 基链接线速度 linear velocity of baselink, in body frame
    double baseAcc[3]{0};       // 基链接加速 acceleration of baselink, in body frame
    double baseAngVel[3]{0};    // 基链接角速度 angular velocity of baselink, in body frame

    // 预定义的关节名称列表（应该与 XML 模型中的关节名对应）
    const std::vector<std::string> JointName = {
        "leg1_1_joint", "leg1_2_joint", "leg1_3_joint",
        "leg2_1_joint", "leg2_2_joint", "leg2_3_joint",
        "leg3_1_joint", "leg3_2_joint", "leg3_3_joint",
        "leg4_1_joint", "leg4_2_joint", "leg4_3_joint"
    };
    const std::string baseName="base_link";                 // 基链接名称
    const std::string orientationSensorName = "imu_quat";   // 四元数名称
    const std::string gyroSensorName = "imu_gyro";          // 陀螺仪名称
    const std::string accSensorName = "imu_acc";            // 加速度计名称
    const std::string velSensorName="baselink-velocity";    // 速度传感器名称

    // 初始化 mj_model 和 mj_data 指针。
    // 设置时间步长 timeStep。
    // 确定关节数量 jointNum。
    // 初始化存储关节索引和状态的向量。
    // 查找每个关节、传感器和基链接的 ID，并检查是否在 XML 文件中存在。如果不存在，则输出错误信息并终止程序。
    MJ_Interface(mjModel *mj_modelIn, mjData  *mj_dataIn);
    //更新传感器数据
    void updateSensorValues();
    // 将输入的扭矩向量 tauIn 应用到 MuJoCo 的控制数组中，从而控制各个关节的运动。
    void setMotorsTorque(std::vector<double> &tauIn);
   
private:
    mjModel *mj_model;  // MuJoCo 模型指针
    mjData  *mj_data;   // MuJoCo 数据指针
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl;// 关节位置、速度、控制索引
    int orientataionSensorId;// 四元数传感器 ID
    int velSensorId;// 速度传感器 ID
    int gyroSensorId;// 陀螺仪传感器 ID
    int accSensorId;// 加速度计传感器 ID
    int baseBodyId;// 基链接 ID
    double timeStep{0.001}; // 仿真时间步长
};





#endif