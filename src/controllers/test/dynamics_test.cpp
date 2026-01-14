
#include "Dynamics.h" // 请根据你的实际头文件路径修改

#include <ros/ros.h>
#include <ros/package.h> // 用于获取包路径
#include <chrono> // 用于微秒级计时

using namespace controllers; // 使用你的命名空间

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamics_test_node");
    ros::NodeHandle nh;

    // 1. 实例化动力学类
    Dynamics dyn;
    Robot_info robot_data;

    // 2. 获取 URDF 路径并初始化
    // 假设你的 urdf 放在 robot_description 包的 urdf 文件夹下，名字叫 robot.urdf
    // 如果不是，请手动修改 path
    std::string urdf_pkg_dir = ros::package::getPath("robot_description");//urdf包路径
    std::string urdf_path = urdf_pkg_dir + "/robot/urdf/robot.urdf";//urdf路径
    ROS_INFO("succes urdf_path: %s", urdf_path.c_str());

    dyn.init_model(urdf_path);

    // 3. 初始化数据结构 (避免空数据)
    robot_data.motor_q = Eigen::VectorXd::Zero(12);
    robot_data.motor_dq = Eigen::VectorXd::Zero(12);
    robot_data.base_pos.setZero();
    robot_data.base_vel.setZero();
    robot_data.imu_quat.setIdentity(); // w=1, x=0, y=0, z=0
    robot_data.imu_gyro.setZero();
    robot_data.imu_acc.setZero();

    // 4. 模拟控制循环 (1000Hz)
    ros::Rate rate(1000);
    double t = 0;

    ROS_INFO("Start Profiling Loop...");

    while (ros::ok()) {
        t += 0.001;

        // --- A. 生成伪造的运动数据 (让机器人动起来) ---
        // 让关节做正弦运动，测试雅可比和速度变化
        for(int i=0; i<12; i++) {
            robot_data.motor_q(i) = 0.5 * sin(t + i*0.1); 
            robot_data.motor_dq(i) = 0.5 * cos(t + i*0.1);
        }
        // 让基座浮动
        robot_data.base_pos << 0, 0, 0.3 + 0.05*sin(t);
        robot_data.imu_gyro << 0.1, 0, 0; // 模拟一点角速度

        // --- B. 核心计算与计时 ---
        auto start = std::chrono::high_resolution_clock::now();
        
        dyn.update_dynamics(robot_data); // <--- 调用你的库
        
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::micro> elapsed = end - start;

        // --- C. 打印检查 (每 1000 次打印一次，避免刷屏) ---
        // 打印耗时和关键数据
        static int count = 0;
        if (count++ % 1000 == 0) {
            ROS_INFO("------------------------------------------------");
            ROS_INFO_STREAM("Calculation Time: " << elapsed.count() << " us (Target: < 100 us)");
            
            // 检查1: 足端位置 (看是否符合几何常识)
            ROS_INFO_STREAM("FL Foot Pos (World): " << robot_data.foot_pos_world[0].transpose());
            
            // 检查2: 质量矩阵对称性 (M - M^T 应该接近 0)
            double symmetry_error = (robot_data.wbc_M - robot_data.wbc_M.transpose()).norm();
            if(symmetry_error < 1e-5) 
                ROS_INFO("Mass Matrix Symmetry: OK");
            else 
                ROS_WARN_STREAM("Mass Matrix Asymmetric! Error: " << symmetry_error);

            // 检查3: 漂移项是否被计算 (不为0)
            ROS_INFO_STREAM("FL Drift Term (dJ*dq): " << robot_data.wbc_dJdq[0].transpose());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}