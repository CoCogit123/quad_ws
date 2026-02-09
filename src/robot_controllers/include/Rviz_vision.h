#ifndef RVIZ_VISION_H
#define RVIZ_VISION_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace controllers {

    class Rviz_vision {
    public:
        /**
         * @brief 构造函数
         * @param nh ROS句柄
         * @param base_frame 机器人基座坐标系名称 (如 "base_link")
         * @param world_frame 世界坐标系名称 (如 "odom" or "map")
         */
        Rviz_vision(ros::NodeHandle& nh, std::string base_frame, std::string world_frame) 
            : nh_(nh), base_frame_(base_frame), world_frame_(world_frame) {
            
            // 1. 初始化发布者
            joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_markers", 10);

            // 2. 定义12个电机的名称 (必须与你的URDF文件中的 joint name 一致)
            // 顺序通常是: LF_HAA, LF_HFE, LF_KFE, RF..., LH..., RH... (具体需根据你的URDF调整)
            joint_names_ = {
                "leg1_joint1", "leg1_joint2", "leg1_joint3",
                "leg2_joint1", "leg2_joint2", "leg2_joint3",
                "leg3_joint1", "leg3_joint2", "leg3_joint3",
                "leg4_joint1", "leg4_joint2", "leg4_joint3"
            };
        }

        /**
         * @brief 功能1: 更新机器人位姿和关节角度
         * @param position 机器人在世界坐标系下的位置 (x, y, z)
         * @param orientation 机器人的姿态四元数 (w, x, y, z)
         * @param joint_angles 12个电机的角度 (弧度制)
         */
        void updateRobotState(const Eigen::Vector3d& position, 
                            const Eigen::Quaterniond& orientation, 
                            const std::vector<double>& joint_angles);

        /**
         * @brief 功能2: 显示摆动腿的关键点 (起点、中点、落足点)
         * @param start_pos  摆动起点 (世界坐标系)
         * @param mid_pos    摆动最高点/中点 (世界坐标系)
         * @param target_pos 目标落足点 (世界坐标系)
         * @param leg_id     腿的ID (用于区分Marker ID, 0-3)
         */
        void visualizeSwingLeg(const Eigen::Vector3d& start_pos, 
                            const Eigen::Vector3d& mid_pos, 
                            const Eigen::Vector3d& target_pos,
                            int leg_id);

        /**
         * @brief 在Rviz中显示一个点（以球体Marker形式）
         * @param position 相对坐标系下的位置向量 (Eigen::Vector3d)
         * @param frame_id 相对坐标系的名称 (如 "base_link" 或 "odom")
         * @param point_id 用于区分不同的点，避免互相覆盖
         */
        void point_to_link(const Eigen::Vector3d& position, const std::string& frame_id, int point_id);

    private:
        ros::NodeHandle nh_;
        ros::Publisher joint_pub_;
        ros::Publisher marker_pub_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        
        std::string base_frame_;
        std::string world_frame_;
        std::vector<std::string> joint_names_;
    };

}
#endif