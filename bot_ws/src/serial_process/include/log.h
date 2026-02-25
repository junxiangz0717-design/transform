// Created by zh on 24-8-3.
// Update by tzz on 25-3-14.
//话题错误记录
#include <fstream>
#include <iostream>
#include <string>

#pragma once
using namespace std;
class Log
{
private:
    std::ofstream log_file;
public:
    struct NewLine {};
    static constexpr NewLine nl{};
    explicit Log(const std::string &file_name)
    {
        const std::string &path = file_name;
        log_file.open(path, std::ios_base::app);
        // 检查文件是否成功打开（或创建）
        if (!log_file) {
            std::cerr << "log_file无法打开（或创建）文件以追加内容" << std::endl << std::flush;
            std::cerr << "路径: " << path << std::endl;
            std::cerr << "错误信息: " << std::strerror(errno) << std::endl;
            std::cerr << std::flush;
        }
        else
        {
            // 记录当前日期时间
            // std::time_t now = std::time(nullptr);
            // log_file << std::endl << std::asctime(std::localtime(&now));
            // log_file.flush();
        }
    }
    
    template<class T>
    void operator<< (T msg)
    {
        log_file << msg << flush;
    }
    void operator<< (NewLine)
    {
        log_file << std::endl;
        log_file.flush();
    }

    void error(const std::string& str)
    {
        log_file << "[ERROR] " + str << std::endl;
        log_file.flush();
    };
};
 inline Log log_serial("../log/serial_log.txt");