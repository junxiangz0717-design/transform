/**
 * @file serial_port.cpp
 * @author 杨泽霖 (scut.bigeyoung@qq.com)
 * @brief Linux串口类
 * @version 2.0
 * @date 2018-12-08
 *
 * @copyright Copyright South China Tiger(c) 2018
 *
 */

#include <cstring>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <string>
#include <unistd.h>

#include "serial_logging.h"
#include "serial_port.h"

using namespace std;

/**
 * @brief Open the serial port, search all available devices automatically and try to open the first
 */
bool SerialPort::open()
{
    __is_open = false;

    DIR *dir = nullptr;
    struct dirent *dire = nullptr;
    file_name.clear();
    const char *dir_path = "/dev/";
    if ((dir = opendir(dir_path)) != nullptr)
    {
        while ((dire = readdir(dir)) != nullptr)
        {
            if ((strstr(dire->d_name, open_target.c_str()) != nullptr))
            {
                file_name = dire->d_name;
                break;
            }
        }
        closedir(dir);
    }

    if (file_name.empty())
    {
        SER_ERROR("无法在/dev中找到/*%s*", open_target.c_str());
        return false;
    }
    else
        file_name = dir_path + file_name;

    SER_INFO("正在打开串口: %s……", file_name.c_str());
    __fd = ::open(file_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); // 非堵塞情况

    if (__fd == -1)
    {
        SER_ERROR("打开串口%s失败！", file_name.c_str());
        perror(file_name.c_str());
        return false;
    }
    tcgetattr(__fd, &__option);

    // 修改所获得的参数
    __option.c_iflag = 0;                 // 原始输入模式
    __option.c_oflag = 0;                 // 原始输出模式
    __option.c_lflag = 0;                 // 关闭终端模式
    __option.c_cflag |= (CLOCAL | CREAD); // 设置控制模式状态，本地连接，接收使能
    __option.c_cflag &= ~CSIZE;           // 字符长度，设置数据位之前一定要屏掉这个位
    __option.c_cflag &= ~CRTSCTS;         // 无硬件流控
    __option.c_cflag |= CS8;              // 8位数据长度
    __option.c_cflag &= ~CSTOPB;          // 1位停止位
    __option.c_cc[VTIME] = 0;
    __option.c_cc[VMIN] = 0;
    cfsetospeed(&__option, __baud_rate); // 设置输入波特率
    cfsetispeed(&__option, __baud_rate); // 设置输出波特率

    // 设置新属性，TCSANOW：所有改变立即生效
    tcsetattr(__fd, TCSANOW, &__option);

    __is_open = true;

    SER_PASS("成功打开串口: %s", file_name.c_str());
    return true;
}

/**
 * @brief Close the serial port
 */
void SerialPort::close()
{
    if (__is_open)
        ::close(__fd);
    __is_open = false;
}

/**
 * @brief 8位CRC校验
 *
 * @param _data 数据指针
 * @param length 数据长度
 * @param polynomial 校验码
 * @return uint8_t 校验和
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
 * @brief Write data
 *
 * @param data Start position of the data
 * @param length The length of the data to be written
 * @return Whether to write in full
 */
ssize_t SerialPort::write(void *data, size_t length)
{
    ssize_t len_result = -1;
    if (__is_open)
    {
        tcflush(__fd, TCOFLUSH);                  // 清空，防止数据累积在缓存区，在<termios.h>中
        len_result = ::write(__fd, data, length); // ::write在<unistd.h>中
    }

    if (len_result != static_cast<ssize_t>(length))
    {
        SER_ERROR("无法写入串口%s，重新打开中...", file_name.c_str());
        open();
    }
    else
        SER_WARNING("数据写入%s成功", file_name.c_str());

    return len_result;
}

/**
 * @brief Read data
 *
 * @param data Start position of the data
 * @param len The length of the data to be read
 * @return Length
 */
ssize_t SerialPort::read(void *data, size_t len)
{
    static int failure_times;
    ssize_t len_result = -1;
    /*
        ::read：这里使用了全局作用域解析运算符 ::，
        表示调用全局命名空间中的 read 函数，而不是可能存在于局部作用域的同名函数。
        read 函数是用于从文件描述符中读取数据的 POSIX 函数。
    */
    if (__is_open)
    {
        // 成功读取的字节数赋给 len_result
        len_result = ::read(__fd, data, len);
        tcflush(__fd, TCIFLUSH); // tcflush 是一个与终端设备（terminal device）相关的函数，用于刷新终端设备的输入或输出缓冲区。这个函数通常用于串口通信或终端操作。
    }

    if (len_result == -1)
    {
        failure_times++;
        if (failure_times > 20)
        {
            failure_times = 0;
            // system("rosnode kill /receive_process /send_process");
        }
        SER_ERROR("串口%s不可读取，将重新打开……", file_name.c_str());
        open();
    }
    else if (len_result >0)
    {
        SER_PASS("读取串口%s成功", file_name.c_str());
    }
    return len_result;
}