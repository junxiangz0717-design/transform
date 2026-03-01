/**
 * @file serial_port.h
 * @author 张俊翔(junxiangz0717@gmail.com)
 * @brief 基于 Linux 系统调用的串口通信封装类
 * @date 2026-02-25
 * @copyright Copyright SCUT RobotLab(c) 2026
 */

#pragma once
#include <string>
#include <memory>
#include <termios.h>
#include <vector>
#include <stdio.h>

//! @addtogroup serial_port
//! @{

/**
 * @class SerialPort
 * @brief 串口通信封装类，支持自动设备发现与结构化数据解析
 */
class SerialPort
{
private:
    std::string file_name_;              ///< 实际打开的串口设备路径 (如 /dev/ttyACM0)
    std::string open_target_ = "ttyACM"; ///< 自动搜索设备时的匹配关键字

    termios option_;       ///< 终端配置结构体
    int fd_;               ///< 串口文件描述符
    bool is_open_ = false; ///< 串口状态标志位
    int baud_rate_;        ///< 波特率

public:
    /**
     * @brief 构造函数：初始化并尝试自动循环打开目标串口直到成功
     * @param baud_rate 波特率常量，默认为 B921600
     */
    explicit SerialPort(int baud_rate = B921600) : baud_rate_(baud_rate)
    {
        while (!open())
        {
        }
    }

    /**
     * @brief 析构函数：释放资源并关闭已打开的串口
     */
    ~SerialPort() { close(); }

    /**
     * @brief 自动搜索 /dev/ 目录下匹配的目标设备并尝试初始化
     * @return true 打开成功，false 找不到设备或初始化失败
     */
    bool open();

    /**
     * @brief 关闭当前串口，重置标志位
     */
    void close();

    /**
     * @brief 从串口读取结构体数据，自动解析帧头和帧尾，并进行 CRC8 校验
     * @tparam T 读取到结构体的类型
     * @param head 头帧
     * @param tail 尾帧
     * @return std::vector<T> 包含所有校验通过的结构体集合
     */
    template <typename T>
    std::vector<T> readStruct(uint8_t head, uint8_t tail)
    {
        (void)tail;
        std::vector<T> vec_t;
        constexpr int LENGTH = 512;
        constexpr int SIZE = sizeof(T);
        uint8_t read_buffer[LENGTH] = {0};
        ssize_t len_result = read(read_buffer, LENGTH);
        for (ssize_t i = 0; (i + SIZE + 1) < len_result; i++)
            if (read_buffer[i] == head)
            {
                if (!CRC8(&read_buffer[i], SIZE + 2))
                    vec_t.push_back(*(reinterpret_cast<T *>(&read_buffer[i + 1])));
            }
        return vec_t;
    }

    /**
     * @brief 将结构体数据直接序列化并写入串口
     * @tparam _Tp 写入结构体的类型
     * @param data_struct 结构体引用
     * @return true 写入长度与结构体大小完全一致
     */
    template <typename _Tp>
    bool writeStruct(_Tp &data_struct)
    {
        ssize_t len_result = write(&data_struct, sizeof(data_struct));
        printf("len_result: %d\n", int(len_result));
        return (sizeof(data_struct) == len_result);
    }

    /**
     * @brief 检查串口是否处于有效打开状态
     * @return true 已打开
     */
    inline bool isOpened() const { return is_open_; };

private:
    /**
     * @brief 底层写操作封装
     * @param data 数据起始地址
     * @param len 期望写入的长度
     * @return ssize_t 实际写入的字节数
     */
    ssize_t write(void *data, size_t len);

    /**
     * @brief 底层读操作封装
     * @param data 数据起始地址
     * @param len 期望读取的长度
     * @return ssize_t 实际读取的字节数
     */
    ssize_t read(void *data, size_t len);

    /**
     * @brief 8位循环冗余校验 (CRC-8)
     * @param _data 校验数据指针
     * @param length 校验长度
     * @param polynomial 校验多项式，默认 0x31
     * @return uint8_t 校验结果 (0 表示校验通过)
     */
    uint8_t CRC8(const void *_data, uint16_t length, uint8_t polynomial = 0x31);
};

using serial_ptr = std::unique_ptr<SerialPort>;

//! @} serial_port
