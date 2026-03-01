/**
 * @file serial_port.cpp
 * @author 张俊翔(junxiangz0717@gmail.com)
 * @brief 串口设备的自动发现、打开配置以及读写封装实现
 * @date 2026-02-25
 * @copyright Copyright SCUT RobotLab(c) 2026
 */

#include <cstring>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

#include "serial_port.h"

using namespace std;

/**
 * @brief 自动搜索并打开匹配的串口设备
 * @details 遍历 /dev/ 目录，寻找名称包含 open_target_ (如 ttyACM) 的第一个设备并配置波特率。
 * @return true 成功打开并配置，false 找不到设备或配置失败
 */
bool SerialPort::open()
{
    is_open_ = false;

    DIR *dir = nullptr;
    struct dirent *dire = nullptr;
    file_name_.clear();
    const char *dir_path = "/dev/";

    // 1. 扫描设备目录，寻找匹配的串口设备
    if ((dir = opendir(dir_path)) != nullptr)
    {
        while ((dire = readdir(dir)) != nullptr)
        {
            if ((strstr(dire->d_name, open_target_.c_str()) != nullptr))
            {
                file_name_ = dire->d_name;
                break;
            }
        }
        closedir(dir);
    }

    if (file_name_.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("serial_port"), "无法在 /dev 中找到匹配 '%s' 的设备", open_target_.c_str());
        return false;
    }
    else
        file_name_ = dir_path + file_name_;

    RCLCPP_INFO(rclcpp::get_logger("serial_port"), "正在打开串口: %s ...", file_name_.c_str());

    // 2. 调用系统底层 open 使用 O_NOCTTY 防止设备成为控制终端，O_NDELAY 启用非阻塞模式
    fd_ = ::open(file_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd_ == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("serial_port"), "打开串口 %s 失败: %s", file_name_.c_str(), strerror(errno));
        return false;
    }

    // 3. 配置 termios 属性
    tcgetattr(fd_, &option_);

    // 修改所获得的参数
    option_.c_iflag = 0;                 // 原始输入模式
    option_.c_oflag = 0;                 // 原始输出模式
    option_.c_lflag = 0;                 // 关闭终端模式
    option_.c_cflag |= (CLOCAL | CREAD); // 设置控制模式状态，本地连接，接收使能
    option_.c_cflag &= ~CSIZE;           // 字符长度，设置数据位之前一定要屏掉这个位
    option_.c_cflag &= ~CRTSCTS;         // 清除字符长度位
    option_.c_cflag |= CS8;              // 8位数据长度
    option_.c_cflag &= ~CSTOPB;          // 1位停止位
    option_.c_cc[VTIME] = 0;             // 非规范模式下的读取超时
    option_.c_cc[VMIN] = 0;              // 非规范模式下的最小读取字符

    cfsetospeed(&option_, baud_rate_); // 设置输出波特率
    cfsetispeed(&option_, baud_rate_); // 设置输入波特率

    // 4. 应用设置 TCSANOW 表示立即生效
    if (tcsetattr(fd_, TCSANOW, &option_) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("serial_port"), "设置串口属性失败！");
        return false;
    }

    is_open_ = true;

    RCLCPP_INFO(rclcpp::get_logger("serial_port"), "成功打开串口: %s", file_name_.c_str());
    return true;
}

/**
 * @brief 关闭串口并清理文件描述符
 */
void SerialPort::close()
{
    if (is_open_)
    {
        ::close(fd_);
        is_open_ = false;
        RCLCPP_INFO(rclcpp::get_logger("serial_port"), "串口 %s 已关闭", file_name_.c_str());
    }
}

/**
 * @brief 8位 CRC 校验算法
 * @param _data 数据起始指针
 * @param length 需要校验的长度
 * @param polynomial 校验码，默认 0x31
 * @return uint8_t 校验和 (结果为 0 通常表示校验成功)
 */
uint8_t SerialPort::CRC8(const void *_data, uint16_t length, uint8_t polynomial)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= ((uint8_t *)_data)[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x1)
            {
                crc = (crc >> 1) ^ polynomial;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

/**
 * @brief 向串口写入原始字节流
 * @param data 发送数据指针
 * @param length 发送长度
 * @return ssize_t 实际写入长度，失败返回 -1
 */
ssize_t SerialPort::write(void *data, size_t length)
{
    ssize_t len_result = -1;
    if (is_open_)
    {
        tcflush(fd_, TCOFLUSH); // 清空输出缓冲区，确保数据实时性
        len_result = ::write(fd_, data, length);
    }

    if (len_result != static_cast<ssize_t>(length))
    {
        RCLCPP_ERROR(rclcpp::get_logger("serial_port"), "无法写入串口%s，重新打开中...", file_name_.c_str());
        open();
    }
    else
        RCLCPP_WARN(rclcpp::get_logger("serial_port"), "数据写入%s成功", file_name_.c_str());

    return len_result;
}

/**
 * @brief 从串口读取原始字节流
 * @param data 接收缓冲区指针
 * @param len 期望读取长度
 * @return ssize_t 实际读取长度
 */
ssize_t SerialPort::read(void *data, size_t len)
{
    static int failure_times;
    ssize_t len_result = -1;

    if (is_open_)
    {
        len_result = ::read(fd_, data, len);
        tcflush(fd_, TCIFLUSH); // 刷新输入缓冲区
    }

    if (len_result == -1)
    {
        failure_times++;
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("serial_port"), *rclcpp::Clock::make_shared(), 1000,
                              "串口 %s 不可读取，错误原因: %s", file_name_.c_str(), strerror(errno));
        if (failure_times > 20)
        {
            failure_times = 0;
            // system("rosnode kill /receive_process /send_process");
        }
        open();
    }
    else if (len_result == 0)
    {
        failure_times++;
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("serial_port"), *rclcpp::Clock::make_shared(), 1000,
                             "读取串口 %s 内容为空", file_name_.c_str());
        if (failure_times > 20)
        {
            failure_times = 0;
            // system("rosnode kill /receive_process /send_process");
        }
        // open();
    }
    else
        RCLCPP_INFO(rclcpp::get_logger("serial_port"), "读取串口%s成功", file_name_.c_str());
    return len_result;
}