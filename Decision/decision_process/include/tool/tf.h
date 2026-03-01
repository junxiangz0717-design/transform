//
// Created by zh on 24-8-2.
// Upadate by tzz on 25-1-17

#pragma once
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <array>

// 将四元数转换为yaw角度的通用函数
double quaternion_to_yaw(const tf2::Quaternion& q)
{
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

// 从nav_msgs::Odometry中提取四元数并转换为yaw角度
void orientation_to_yaw(const nav_msgs::msg::Odometry::SharedPtr& odom_data, double& yaw)
{
    tf2::Quaternion q(odom_data->pose.pose.orientation.x,
                      odom_data->pose.pose.orientation.y,
                      odom_data->pose.pose.orientation.z,
                      odom_data->pose.pose.orientation.w);
    yaw = quaternion_to_yaw(q);
}

// 从nav_msgs::Odometry中提取四元数并返回yaw角度
double orientation_to_yaw(const nav_msgs::msg::Odometry::SharedPtr& odom_data)
{
    tf2::Quaternion q(odom_data->pose.pose.orientation.x,
                      odom_data->pose.pose.orientation.y,
                      odom_data->pose.pose.orientation.z,
                      odom_data->pose.pose.orientation.w);
    return quaternion_to_yaw(q);
}

// 从geometry_msgs::Quaternion中提取四元数并返回yaw角度
double orientation_to_yaw(const geometry_msgs::msg::Quaternion& orientation)
{
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    return quaternion_to_yaw(q);
}

std::array<double, 4> yawToQuaternion(const double& yaw) 
{
    // 计算四元数的各个分量
    double w = std::cos(yaw / 2);
    double x = 0;
    double y = 0;
    double z = std::sin(yaw / 2);

    // 使用 std::array 存储四元数
    return {w, x, y, z};
}