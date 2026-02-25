/**
 * @file serial_port.h
 * @author 杨泽霖 (scut.bigeyoung@qq.com)
 *         赵曦 (535394140@qq.com)
 * @brief
 * @version 1.0
 * @date 2022-09-27
 *
 * @copyright Copyright SCUT RobotLab(c) 2022
 *
 */

#pragma once
#include <string>
#include <memory>
#include <termios.h>
#include <vector>
#include <stdio.h>

/*
<memory>: 这个头文件提供了与内存分配、智能指针和其他与内存相关的工具有关的类和函数。例如，它包含了 std::unique_ptr 和 std::shared_ptr 等智能指针类的定义，用于管理动态分配的内存。
<termios.h>: 这是一个特定于 POSIX 系统的头文件，包含了与终端 I/O（输入/输出）有关的函数和常量。其中定义了与终端设置和控制相关的结构体和函数，例如 struct termios 用于表示终端属性。
<vector>: 这个头文件包含了 C++ 标准库中的 std::vector 类的定义。std::vector 是一个动态数组，可以在运行时调整大小，提供了方便的操作接口和动态内存管理。
*/

//! @addtogroup serial_port
//! @{

/**
 * @brief Serial communication library
 * 两个下划线__的出发点是想区分公有和私有变量，但是最好不要用这种方式，属于未定义行为，程序非良构。
 */
class SerialPort
{
    std::string file_name;              // 实际打开的串口设备名称
    std::string open_target = "ttyACM"; // 用于匹配的目标打开串口名称

    termios __option;       //!< 终端控制
    int __fd;               //!< 文件描述符
    bool __is_open = false; //!< 串口打开标志位
    int __baud_rate;        //!< 波特率

public:
    /**
     * @brief 构造串口对象时，重复试打开目标串口直到打开成功
     *
     * @param baud_rate 波特率，默认B 200 0000
     */
    explicit SerialPort(int baud_rate = B921600) : __baud_rate(baud_rate)
    { while(!open()){} }

    /**
     * @brief 析构时关闭串口
     */
    ~SerialPort() { close(); }

    /**
     * @brief Open the serial port, search all available devices automatically and try to open the first
     */
    bool open();

    /**
     * @brief 关闭串口
     */
    void close();

    /**
     * @brief 从串口读取结构体
     *
     * @tparam T 读取到结构体的类型
     * @param head 头帧
     * @param tail 尾帧
     * @return std::vector<T> 读到的所有结构体
     */
    template <typename T>
    std::vector<T> readStruct(unsigned char head, unsigned char tail)
    {
        std::vector<T> vec_t;
        constexpr int LENGTH = 512; // 使用 constexpr 关键字将其声明为一个编译时计算的常量，意味着编译器会在编译时确定其值，并在编译过程中使用该值。
        constexpr int SIZE = sizeof(T);
        unsigned char read_buffer[LENGTH] = {0};
        ssize_t len_result = read(read_buffer, LENGTH); // ssize_t 32或63位有符号整型，用于表示可以返回负数的整型
        for (ssize_t i = 0; (i + SIZE + 1) < len_result; i++)
            if (read_buffer[i] == head)
            {
                if (!CRC8(&read_buffer[i], SIZE + 2)) // head和tail只占一个T的长度
                    vec_t.push_back(*(reinterpret_cast<T *>(&read_buffer[i + 1])));
            }
        return vec_t;
    }

    /**
     * @brief Struct data write to serial port
     *
     * @tparam _Tp 写入结构体的类型
     * @param data_struct 要写入的结构体
     * @return 是否写入成功
     *
     * @brief The serial is opened?
     */
    template <typename _Tp>
    bool writeStruct(_Tp &data_struct)
    {
        ssize_t len_result = write(&data_struct, sizeof(data_struct));
        // printf("len_result: %d\n", int(len_result));
        return (sizeof(data_struct) == len_result);
    }

    /*
     * @return true
     * @return false
     */
    inline bool isOpened() const { return __is_open; };

private:
    /**
     * @brief Write data
     *
     * @param data Start position of the data
     * @param length The length of the data to be written
     * @return Whether to write in full
     */
    ssize_t write(void *data, size_t len); // 返回成功写入的字节数

    /**
     * @brief Read data
     *
     * @param data Start position of the data
     * @param len The length of the data to be read
     * @return Length
     */
    ssize_t read(void *data, size_t len); // 返回成功读取的字节数

    uint8_t CRC8(const void *_data, uint16_t length, uint8_t polynomial = 0x31);
};

using serial_ptr = std::unique_ptr<SerialPort>;

//! @} serial_port
