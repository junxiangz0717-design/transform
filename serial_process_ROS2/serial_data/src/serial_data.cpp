/**
 * @file serial_data.h
 * @author 张俊翔(junxiangz0717@gmail.com)
 * @brief  哨兵机器人串口通信协议的具体实现，包含数据读取、封装发送及 CRC 校验
 * @date 2026-02-25
 * @copyright Copyright SCUT RobotLab(c) 2026
 */

#include "serial_data.h"
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace std;

ReceiveData SerialData::zh_read()
{
    // 使用帧头 0x66 和帧尾 0x55 从串口底层读取结构体数组
    vector<ReceiveData> receive_datas = serial_port_.readStruct<ReceiveData>(0x66, 0x55);
    RCLCPP_INFO(rclcpp::get_logger("serial_data"), "Serial port reading...");
    
    if (!receive_datas.empty())
        return receive_datas.back();
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("serial_data"), "串口读取的字节中不含有哨兵运动数据包，将返回空数据包！");
        return {};
    }
}

void SerialData::zh_write(SendData send_data)
{
    static uint8_t self_add_flag = 0;
    if (self_add_flag == 255)
    {
        self_add_flag = 0;
    }
    self_add_flag++;
    
    send_data.self_add_flag = self_add_flag;
    send_data.end = CRC16(&send_data, sizeof(SendData) - 2);
    
    // 写入串口驱动
    if (!serial_port_.writeStruct<SendData>(send_data)) {
        RCLCPP_ERROR(rclcpp::get_logger("serial_data"), "写入串口失败！");
    }
}

/**
 * @brief 8位CRC校验
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