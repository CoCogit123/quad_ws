#include "MJ_interface.h"


MJ_Interface::MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn)
{
    mj_model = mj_modelIn;  // 将传入的 MuJoCo 模型指针赋值给类成员 mj_model
    mj_data  = mj_dataIn;    // 将传入的 MuJoCo 数据指针赋值给类成员 mj_data
    timeStep = mj_model->opt.timestep;  // 从模型中获取仿真时间步长
    jointNum = JointName.size();        // 获取预定义的关节名称列表长度，即关节数量
    //// 初始化存储关节索引和控制参数的向量
    jntId_qpos.assign(jointNum, 0);
    jntId_qvel.assign(jointNum, 0);
    jntId_dctl.assign(jointNum, 0);
    motor_pos.assign(jointNum, 0);
    motor_vel.assign(jointNum, 0);
    motor_pos_Old.assign(jointNum, 0);
    for (int i = 0; i < jointNum; i++)
    {
        // 根据关节名称查找关节在模型中的 ID
        int tmpId = mj_name2id(mj_model, mjOBJ_JOINT, JointName[i].c_str());
        if (tmpId == -1)// 若 ID 为 -1，说明关节未在 XML 模型中找到
        {
            std::cerr << JointName[i] << " not found in the XML file!" << std::endl;
            std::terminate();
        }
        // 存储关节位置（qpos）和速度（qvel）的地址偏移
        jntId_qpos[i] = mj_model->jnt_qposadr[tmpId];
        jntId_qvel[i] = mj_model->jnt_dofadr[tmpId];
        // 解析电机名称（从关节名中提取电机名）
        std::string motorName = JointName[i].substr(0, JointName[i].find("_joint"));
        tmpId = mj_name2id(mj_model, mjOBJ_ACTUATOR, motorName.c_str());
        if (tmpId == -1)
        {
            std::cerr << motorName << " not found in the XML file!" << std::endl;
            std::terminate();
        }
        jntId_dctl[i] = tmpId;
    }
    // 查找基链接（base_link）的 ID
    baseBodyId = mj_name2id(mj_model, mjOBJ_BODY, baseName.c_str());
    // 查找各类传感器的 ID
    orientataionSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, orientationSensorName.c_str());
    velSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, velSensorName.c_str());
    gyroSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, gyroSensorName.c_str());
    accSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, accSensorName.c_str());
}

void MJ_Interface::updateSensorValues()
{
    //电机角度，电机速度
    for (int i = 0; i < jointNum; i++)
    {
        motor_pos_Old[i] = motor_pos[i];                // 保存上一时刻关节位置
        motor_pos[i] = mj_data->qpos[jntId_qpos[i]];    // 获取当前关节位置
        motor_vel[i] = mj_data->qvel[jntId_qvel[i]];    // 获取当前关节速度
    }
    //四元数
    for (int i = 0; i < 4; i++){// 获取基链接的四元数（MuJoCo 顺序为 [w, x, y, z]）重新排列四元数顺序为 [x, y, z, w]
        baseQuat[i] = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId] + i];
    }
    double tmp = baseQuat[0];
    baseQuat[0] = baseQuat[1];
    baseQuat[1] = baseQuat[2];
    baseQuat[2] = baseQuat[3];
    baseQuat[3] = tmp;

    // 根据四元数计算欧拉角（滚转、俯仰、偏航)
    rpy[0] = atan2(2 * (baseQuat[3] * baseQuat[0] + baseQuat[1] * baseQuat[2]), 1 - 2 * (baseQuat[0] * baseQuat[0] + baseQuat[1] * baseQuat[1]));
    rpy[1] = asin(2 * (baseQuat[3] * baseQuat[1] - baseQuat[0] * baseQuat[2]));
    rpy[2] = atan2(2 * (baseQuat[3] * baseQuat[2] + baseQuat[0] * baseQuat[1]), 1 - 2 * (baseQuat[1] * baseQuat[1] + baseQuat[2] * baseQuat[2]));

    // 处理偏航角（yaw）的连续性，避免角度突变
    if ((rpy[2] - yaw_simgle) > 3.1415926*0.5){
        yaw_N -= 1.0;
    }
    else if ((rpy[2] - yaw_simgle) < -3.1415926*0.5){
        yaw_N += 1.0;
    }
    yaw_simgle=rpy[2];
    rpy[2]=yaw_simgle + yaw_N*2.0*3.1415926;

    // 更新基链接的位置、加速度、角速度和线速度
    for (int i = 0; i < 3; i++)
    {
        double posOld = basePos[i];
        basePos[i] = mj_data->xpos[3 * baseBodyId + i];
        baseLinVel[i] = (basePos[i] - posOld) / (mj_model->opt.timestep);
        baseAcc[i] = mj_data->sensordata[mj_model->sensor_adr[accSensorId] + i];
        baseAngVel[i] = mj_data->sensordata[mj_model->sensor_adr[gyroSensorId] + i];
    }

}

void MJ_Interface::setMotorsTorque(std::vector<double> &tauIn)
{
    for (int i = 0; i < jointNum; i++)
        mj_data->ctrl[i] = tauIn.at(i);
}
















