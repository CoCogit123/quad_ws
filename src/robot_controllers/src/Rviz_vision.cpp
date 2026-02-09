#include "Rviz_vision.h"

namespace controllers {

    void Rviz_vision::updateRobotState(const Eigen::Vector3d& position, 
                            const Eigen::Quaterniond& orientation, 
                            const std::vector<double>& joint_angles) 
    {
        if (joint_angles.size() != 12) {
            ROS_ERROR("Joint angles size mismatch! Expected 12, got %lu", joint_angles.size());
            return;
        }

        ros::Time current_time = ros::Time::now();

        // --- A. 发布 TF (控制机器人在世界中的位置) ---
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = world_frame_;
        transformStamped.child_frame_id = base_frame_;
        
        transformStamped.transform.translation.x = position.x();
        transformStamped.transform.translation.y = position.y();
        transformStamped.transform.translation.z = position.z();
        transformStamped.transform.rotation.w = orientation.w();
        transformStamped.transform.rotation.x = orientation.x();
        transformStamped.transform.rotation.y = orientation.y();
        transformStamped.transform.rotation.z = orientation.z();

        tf_broadcaster_.sendTransform(transformStamped);

        // --- B. 发布 JointState (控制机器人的腿部动作) ---
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = current_time;
        joint_msg.name = joint_names_;
        joint_msg.position = joint_angles;
        
        joint_pub_.publish(joint_msg);
    }

    void Rviz_vision::visualizeSwingLeg(const Eigen::Vector3d& start_pos, 
                            const Eigen::Vector3d& mid_pos, 
                            const Eigen::Vector3d& target_pos,
                            int leg_id) 
    {
        visualization_msgs::MarkerArray marker_array;
        ros::Time current_time = ros::Time::now();

        // 辅助lambda：创建一个球体Marker
        auto createMarker = [&](const Eigen::Vector3d& pos, int id_suffix, float r, float g, float b, const std::string& ns) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = world_frame_;
            marker.header.stamp = current_time;
            marker.ns = ns;
            marker.id = leg_id * 10 + id_suffix; // 确保ID不冲突
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = pos.x();
            marker.pose.position.y = pos.y();
            marker.pose.position.z = pos.z();
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.scale.x = 0.025; // 球体大小 2.5cm
            marker.scale.y = 0.025;
            marker.scale.z = 0.025;
            marker.color.a = 1.0; // 不透明
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            return marker;
        };

        // 1. 起点 (绿色)
        marker_array.markers.push_back(createMarker(start_pos, 1, 0.0, 1.0, 0.0, "swing_start"));
        
        // 2. 中点 (蓝色)
        marker_array.markers.push_back(createMarker(mid_pos, 2, 0.0, 0.0, 1.0, "swing_mid"));
        
        // 3. 落足点 (红色)
        marker_array.markers.push_back(createMarker(target_pos, 3, 1.0, 0.0, 0.0, "swing_end"));

        // 4. (可选) 画一条连接这三点的线，更直观
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = world_frame_;
        line_strip.header.stamp = current_time;
        line_strip.ns = "swing_path";
        line_strip.id = leg_id * 10 + 4;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.pose.orientation.x = 0.0;
        line_strip.pose.orientation.y = 0.0;
        line_strip.pose.orientation.z = 0.0;
        line_strip.pose.orientation.w = 1.0;
        line_strip.scale.x = 0.01; // 线宽
        line_strip.color.a = 0.8;
        line_strip.color.r = 1.0; 
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0; // 黄色轨迹
        
        geometry_msgs::Point p;
        p.x = start_pos.x(); p.y = start_pos.y(); p.z = start_pos.z();
        line_strip.points.push_back(p);
        p.x = mid_pos.x(); p.y = mid_pos.y(); p.z = mid_pos.z();
        line_strip.points.push_back(p);
        p.x = target_pos.x(); p.y = target_pos.y(); p.z = target_pos.z();
        line_strip.points.push_back(p);
        
        marker_array.markers.push_back(line_strip);

        marker_pub_.publish(marker_array);
    }

    /**
     * @brief 在Rviz中显示一个点（以球体Marker形式）
     * @param position 相对坐标系下的位置向量 (Eigen::Vector3d)
     * @param frame_id 相对坐标系的名称 (如 "base_link" 或 "odom")
     * @param point_id 用于区分不同的点，避免互相覆盖
     */
    void Rviz_vision::point_to_link(const Eigen::Vector3d& position, const std::string& frame_id, int point_id) 
    {
        // 修改点：创建一个临时 Array 容器
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "debug_points";
        marker.id = point_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;
        
        marker.color.r = 1.0f; // 改为紫色(1,0,1)，方便和摆动腿的绿蓝红区分
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f; 

        marker.lifetime = ros::Duration(0.1); 

        // 将单体 marker 放入数组
        marker_array.markers.push_back(marker);

        // 使用原本的 marker_pub_ 发布数组
        marker_pub_.publish(marker_array); 
    }


}