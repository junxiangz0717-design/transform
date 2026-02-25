/**
 * @file serial_data.h
 * @author 范文宇 (2643660853@qq.com)、钟华 (1318436194@qq.com)
 * @brief
 * @version 2.0
 * @date 2024-3-19
 *
 * @copyright Copyright (c) 2022
 *
 *
 */
#pragma once
#include "serial_port.h"

// 哨兵视觉接收协议
struct ReceiveData
{
    bool is_path_planning;    // 是否进入路径规划模式标志位
} __attribute__((packed));

/*
__attribute__((packed)) 是一个 GNU 编译器的扩展，用于告诉编译器以紧凑的方式排列结构体的成员，避免因为对齐而浪费内存。
这在某些嵌入式系统或网络通信中可能是必要的，但在一般情况下，结构体的成员通常会被按照编译器的默认对齐规则进行排列。
*/

/**
 * @brief
 *
 */
#pragma pack(1)
struct PathVisual
{
    uint8_t intention = 3u;
    uint16_t start_position_x{};
    uint16_t start_position_y{};
    int8_t delta_x[49]{};
    int8_t delta_y[49]{};
};
#pragma pack()

// 哨兵视觉发送协议
/*
#pragma pack(1) 和 #pragma pack() 是编译器的指令，用于设置结构体的字节对齐方式。
#pragma pack(1) 表示使用紧凑对齐方式，即将结构体成员尽可能地拼凑在一起，不进行字节对齐。
而 #pragma pack() 恢复默认的字节对齐方式。
*/
#pragma pack(1)
struct SendData
{   
    uint8_t head = 0x99;            // 帧头
    float wz;                       // 路径规划模式下的角速度z 
    float vx;                       // 路径规划模式下的线速度x          
    float vy;                       // 路径规划模式下的线速度y
    uint16_t end;          // 尾帧启用CRC16校验
};
#pragma pack()


// 哨兵机器人协议数据包
class SerialData
{
private:
    SerialPort serial_port_{};   // 串口对象

public:
    SerialData() = default;
    
    /**
     * @brief 读取串口
     */
    ReceiveData zh_read();
    
    /**
     * @brief 写入串口
     *
     * @param send_data 待写入数据
     */
    void zh_write(SendData send_data);
    
    static uint16_t CRC16(const void *_data, uint16_t length, uint16_t polynomial = 0x8005);

    static uint8_t CRC8(const void *_data, uint16_t length, uint8_t polynomial = 0x31);
};
inline SerialData serialdata;
