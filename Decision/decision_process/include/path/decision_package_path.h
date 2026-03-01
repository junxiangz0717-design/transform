#pragma once

#include <iostream>
#include <string>
#include <filesystem>
#include <unistd.h> // 包含 realpath 函数
#include <cstdio>   // 包含 FILENAME_MAX 定义

// 检查是否能包含 ament_index_cpp 的头文件
#if defined(__has_include)
#  if __has_include(<ament_index_cpp/get_package_share_directory.hpp>)
#    include <ament_index_cpp/get_package_share_directory.hpp>
#    define HAVE_AMENT_INDEX 1
#  else
#    define HAVE_AMENT_INDEX 0
#  endif
#else
#  define HAVE_AMENT_INDEX 0
#endif

namespace fs = std::filesystem;

// 获取环境变量
inline std::string getEnvVar(const std::string& varName) {
    char* value = getenv(varName.c_str());
    return value != nullptr ? value : "";
}

// 获取可执行文件所在目录
inline std::string getExecutableDirectory() {
    try {
        fs::path executablePath = fs::read_symlink("/proc/self/exe");
        return executablePath.parent_path().string();
    } catch (const fs::filesystem_error& e) {
        std::cerr << "获取可执行文件路径时出错: " << e.what() << std::endl;
        return "";
    }
}

// 尝试返回 package 的 share 目录（优先），找不到则返回空字符串
inline std::string getPackageShareDir()
{
#if HAVE_AMENT_INDEX
    try {
        return ament_index_cpp::get_package_share_directory("decision_process");
    } catch (const std::exception &e) {
        // 回退到可执行相对路径
        return std::string();
    }
#else
    (void)getEnvVar; 
    return std::string();
#endif
}

// 根据 package share 或可执行路径构造常用路径
inline std::string decision_package_path_ros()
{
    auto share = getPackageShareDir();
    if(!share.empty()) return share + "/log/ros_log";
    return getExecutableDirectory() + "/../../../decision_process/log/ros_log";
}

inline std::string decision_package_path_node()
{
    auto share = getPackageShareDir();
    if(!share.empty()) return share + "/log/node_log";
    return getExecutableDirectory() + "/../../../decision_process/log/node_log";
}

inline std::string decision_package_path_xml()
{
    auto share = getPackageShareDir();
    if(!share.empty()) return share + "/XML";
    return getExecutableDirectory() + "/../../../decision_process/XML";
}

inline std::string decision_package_path_semantic_map_xml()
{
    auto share = getPackageShareDir();
    if(!share.empty()) return share + "/include/semantic_map/xml";
    return getExecutableDirectory() + "/../../../decision_process/include/semantic_map/xml";
}

inline std::string yaml_path()
{
    auto share = getPackageShareDir();
    if(!share.empty()) return share + "/config/tree.yaml";
    return getExecutableDirectory() + "/../../../decision_process/config/tree.yaml";
}

inline std::string sentry_pursuit_map_path()
{
    auto share = getPackageShareDir();
    if(!share.empty()) return share + "/include/Pursuit/map/";
    return getExecutableDirectory() + "/../map/";
}

inline std::string pursuit_map_path()
{
    auto share = getPackageShareDir();
    if(!share.empty()) return share + "/map/";
    return getExecutableDirectory() + "/../map/";
}

#define DECISION_PACKAGE_PATH_ROS (decision_package_path_ros())
#define DECISION_PACKAGE_PATH_NODE (decision_package_path_node())
#define DECISION_PACKAGE_PATH_XML (decision_package_path_xml())
#define DECISION_PACKAGE_PATH_SEMNATIC_MAP_XML (decision_package_path_semantic_map_xml())
#define YAML_PATH (yaml_path())
#define SENTRY_PURSUIT_MAP_PATH (sentry_pursuit_map_path())
#define PURSUIT_MAP_PATH (pursuit_map_path())
