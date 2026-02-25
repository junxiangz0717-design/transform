/**
 * @file           : serial_data.cpp
 * @author         : fwy、zh
 * @brief          :
 * @email          : 2643660853@qq.com
 * @date           : 24-3-19
 */

#include "serial_data.h"
#include "serial_logging.h"
#include <iostream>
// #include <ros/ros.h>

using namespace std;

ReceiveData SerialData::zh_read()
{
    // 接收数据
    vector<ReceiveData> receive_datas = serial_port_.readStruct<ReceiveData>(0x66, 0x55); // 0x66, 0x55是帧头和帧尾
    // ROS_INFO("zh_read");
    std::cout<<"zh_read"<<std::endl;
    // 判空
    if (!receive_datas.empty())
        return receive_datas.back();
    else
    {
        SER_ERROR("串口读取的字节中不含有哨兵运动数据包，将返回空数据包！");
        return {};
    }
}

void SerialData::zh_write(SendData send_data)
{
    // static uint8_t self_add_flag = 0;
    // // 一直加一，可以让电控看到有没有刷新
    // if (self_add_flag == 255)
    // {
    //     self_add_flag = 0;
    // }
    // self_add_flag++;
    
    // send_data.self_add_flag = self_add_flag;
    send_data.end = CRC16(&send_data, sizeof(SendData) - 2);
    
    // 写入串口
    serial_port_.writeStruct<SendData>(send_data);
}

/**
 * @brief 8位CRC校验
 *
 * @param _data 数据指针
 * @param length 数据长度
 * @param polynomial 校验码
 * @return uint8_t 校验和
 */
uint8_t SerialData::CRC8(const void *_data, uint16_t length, uint8_t polynomial)
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
 * @brief 16位CRC校验
 *
 * @param _data 数据指针
 * @param length 数据长度
 * @param polynomial 校验码
 * @return uint16_t 校验和
 */
uint16_t SerialData::CRC16(const void *_data, uint16_t length, uint16_t polynomial)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
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