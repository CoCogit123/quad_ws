#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>
#include <iostream>

//自定义消息包
#include <custom_msgs/Joy_control.h> 

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_control");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<custom_msgs::Joy_control>("joy_control", 10);

    // 1. 打开键盘设备 (根据你的系统修改 eventX)
    const char* dev = "/dev/input/event1"; 
    int fd = open(dev, O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        ROS_ERROR("Failed to open keyboard device. Try: sudo chmod 666 %s", dev);
        return 1;
    }

    // 2. 初始化控制变量
    custom_msgs::Joy_control msg;
    msg.run_flag = false;
    msg.yaw_vel = 0.0;
    msg.com_vel.x = 0.0;
    msg.com_vel.y = 0.0;
    msg.com_vel.z = 0.0;
    msg.mode = 0;

    // --- 速度控制参数 ---
    double target_vx = 0.0, target_vy = 0.0, target_vz = 0.0;
    const double MAX_VEL = 0.5;      // 最大恒定速度
    const double RAMP_STEP = 0.02;   // 步进值（值越小越平滑，0.02 在 100Hz 下代表 0.5秒从0加到1.0）

    struct input_event ev;

    ROS_INFO("Global Joy Node Started.");
    ROS_INFO("WS/AD: Move | UH: Height | O: Toggle Run | Zxcv: Modes");

    ros::Rate loop_rate(100); // 100Hz 发布频率
    bool cmd_enable = false;
    //wasd控制速度 uh控制高度 i控制键盘读取 o控制flag zxcv控制模式
    while (ros::ok()) {
        // 2. 捕捉按键（更新目标速度）
        while (read(fd, &ev, sizeof(struct input_event)) > 0) {
            if (ev.type == EV_KEY) {
                int code = ev.code;
                int val = ev.value; // 1:按下, 0:抬起, 2:长按

                if (val == 1&&code == KEY_I)
                {
                    cmd_enable = !cmd_enable;
                    ROS_INFO("Cmd_Enable: %s", cmd_enable ? "ON" : "OFF");
                }

                if(cmd_enable)
                {
                    // 类别 A: 电平触发更新目标速度
                    if (code == KEY_W) target_vx = (val > 0) ?  MAX_VEL : 0.0;
                    else if (code == KEY_S) target_vx = (val > 0) ? -MAX_VEL : 0.0;
                    
                    if (code == KEY_D) target_vy = (val > 0) ?  MAX_VEL : 0.0;
                    else if (code == KEY_A) target_vy = (val > 0) ? -MAX_VEL : 0.0;

                    if (code == KEY_U) target_vz = (val > 0) ?  MAX_VEL : 0.0;
                    else if (code == KEY_H) target_vz = (val > 0) ? -MAX_VEL : 0.0;

                    // 类别 B: 下降沿触发切换
                    if (val == 1) {
                        if (code == KEY_O) {
                            msg.run_flag = !msg.run_flag;
                            ROS_INFO("Run Flag: %s", msg.run_flag ? "ON" : "OFF");
                        }
                        else if (code == KEY_Z) msg.mode = 0;
                        else if (code == KEY_X) msg.mode = 1;
                        else if (code == KEY_C) msg.mode = 2;
                        else if (code == KEY_V) msg.mode = 3;
                        
                        if (code >= KEY_Z && code <= KEY_V) ROS_INFO("Mode: %d", msg.mode);
                    }
                }
            }
        }

        // 3. 速度斜坡处理 (Ramp Logic)
        // 计算当前速度与目标速度的差值，并按步长逼近
        auto apply_ramp = [&](double& current, double target) {
            if (current < target)      current = std::min(current + RAMP_STEP, target);
            else if (current > target) current = std::max(current - RAMP_STEP, target);
        };

        apply_ramp(msg.com_vel.x, target_vx);
        apply_ramp(msg.com_vel.y, target_vy);
        apply_ramp(msg.com_vel.z, target_vz);

        // 4. 定时发布
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    close(fd);
    return 0;
}