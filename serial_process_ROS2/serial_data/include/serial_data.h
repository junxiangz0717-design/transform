/**
 * @file serial_data.h
 * @author 张俊翔(junxiangz0717@gmail.com)
 * @brief 哨兵机器人串口通信协议定义
 * @date 2026-02-25
 * @copyright Copyright SCUT RobotLab(c) 2026
 */

#pragma once
#include "serial_port.h"

/**
 * @brief 哨兵视觉接收协议结构体 (从电控/裁判系统接收)
 * @note 使用 __attribute__((packed)) 确保字节对齐一致，总长度固定
 */
struct ReceiveData
{
    // --- 运动与姿态数据 ---
    float goalx;         ///< 决策层/云台手给定的目标点 x 坐标
    float goaly;         ///< 决策层/云台手给定的目标点 y 坐标
    float delta_yaw;     ///< 大小yaw差值
    float chassis_power; ///< 底盘实时功率 (W)
    float vx;            ///< 当前实时线速度 x
    float vy;            ///< 当前实时线速度 y

    // --- 比赛基础信息 ---
    float time;          ///< 当前比赛阶段剩余时间 (s)
    uint16_t hp_sentry;  ///< 己方哨兵机器人当前血量
    uint8_t color;       ///< 己方颜色: 0-红, 1-蓝
    uint16_t money;      ///< 己方剩余金币/兑换点数量
    uint8_t game_period; ///< 比赛阶段: 0-未开始, 1-准备阶段, 2-自检, 3-倒计时, 4-战斗, 5-结算
    uint8_t style;       ///< 比赛风格建议: 0-保守, 1-进攻, 2-巡逻

    // --- 己方队友坐标 (单位: m) ---
    float our_hero_x;
    float our_hero_y;
    float our_engineer_x;
    float our_engineer_y;
    float our_foot_3_x;
    float our_foot_3_y;
    float our_foot_4_x;
    float our_foot_4_y;

    // --- 血量信息 (HP) ---
    uint16_t our_hero_hp;       ///< 我方英雄血量 (0表示未上场)
    uint16_t our_engineer_hp;   ///< 我方工程血量
    uint16_t our_foot_3_hp;     ///< 我方3号步兵血量
    uint16_t our_foot_4_hp;     ///< 我方4号步兵血量
    uint16_t enemy_hero_hp;     ///< 敌方英雄血量
    uint16_t enemy_foot_3_hp;   ///< 敌方3号步兵血量
    uint16_t enemy_foot_4_hp;   ///< 敌方4号步兵血量
    uint16_t enemy_sentry_hp;   ///< 敌方哨兵血量
    uint16_t enemy_engineer_hp; ///< 敌方工程血量

    // --- 建筑血量 ---
    uint16_t hp_base;          ///< 己方基地血量
    uint16_t hp_outpost;       ///< 己方前哨站血量
    uint16_t hp_enemy_outpost; ///< 敌方前哨站血量
    uint16_t hp_enemy_base;    ///< 敌方基地血量

    // --- 状态与增益 (Buff) ---
    uint8_t add_area_state; ///< 补给区占领状态: 1-已占领 (RMUL适用)
    uint8_t is_center_area; ///< 中心增益点: 0-未被占, 1-己方占, 2-对方占, 3-双方互占 (RMUL)
    uint8_t base_defence;   ///< 己方基地虚拟护盾剩余百分比 (0-100)
    uint8_t add_hp_buff;    ///< 机器人回血增益百分比 (每秒恢复血量上限的 N%)
    uint8_t defence_buff;   ///< 防御增益状态
    uint8_t remain_energy;  ///< 剩余能量状态

    // --- 系统与硬件反馈 ---
    uint32_t RFID;                 ///< RFID 状态位 (RMUC 哨兵巡逻区检测)
    uint32_t event;                ///< 场地事件汇总位
    uint16_t heat_1;               ///< 17mm 枪口 1 实时热量
    uint16_t heat_2;               ///< 17mm 枪口 2 实时热量
    uint16_t ammo;                 ///< 17mm 弹丸允许发弹量 (裁判系统剩余授权)
    uint8_t is_no_ammo;            ///< 空弹检测标志: 1-拨盘空转两圈，确认为无弹
    uint8_t restart_mapping;       ///< 重启导航建图标志位

    // --- 雷达标定信息 ---
    float radar_data[12]; ///< 雷达标定数组（与 msg_process/ReceiveData.msg 对齐）

    // --- 电控保存 ---
    float x_data;
	float y_data;
	float rotationState;
	float temp_x;
	float temp_y;
	float temp_theta;
	float speed_X;
	float speed_Y;
	float speed_Z;
	uint8_t wheel_state1;
	uint8_t wheel_state2;
	uint8_t wheel_state3;
	uint8_t wheel_state4;


    // --- 预留与扩展 ---
    uint32_t sentry_info;  ///< 哨兵自定义信息位 1
    uint16_t sentry_info2; ///< 哨兵自定义信息位 2

} __attribute__((packed));

/**
 * @brief 哨兵视觉发送协议结构体
 */
#pragma pack(1)
struct SendData
{
    uint8_t head = 0x99;            ///< 帧头 (固定为 0x99)
    uint8_t spin_mode;              ///< 底盘小陀螺模式控制
    float decision_yaw_az;          ///< 决策控制云台转动的相对 Yaw 角速度 (逆时针为正)
    float vx;                       ///< 路径规划给出的线速度 x (m/s)
    float vy;                       ///< 路径规划给出的线速度 y (m/s)
    uint8_t style;                  ///< 当前执行的比赛风格: 0-保守, 1-进攻, 2-巡逻
    uint32_t auto_decision_package; ///< 发送给裁判系统的自主决策数据包
    uint8_t pitch_mode = 0U;        ///< Pitch 轴预设档位: 0-平射, 1-高位, 2-低位
    uint8_t attack_rune;            ///< 打符指令位

    uint8_t self_add_flag; ///< 0-255自增标志位，用于电控判断包有无更新
    uint16_t end;          ///< 帧尾: 包含 CRC16 校验值
};
#pragma pack()

/**
 * @brief 串口协议处理类
 */
class SerialData
{
private:
    SerialPort serial_port_{}; ///< 封装的底层 Linux 串口对象

public:
    SerialData() = default;

    /**
     * @brief 从串口读取并解析 ReceiveData
     * @return ReceiveData 解析后的完整接收数据包
     */
    ReceiveData zh_read();

    /**
     * @brief 封装并向串口写入 SendData
     * @param send_data 待写入的结构体数据
     */
    void zh_write(SendData send_data);

    static uint16_t CRC16(const void *_data, uint16_t length, uint16_t polynomial = 0x8005);
    static uint8_t CRC8(const void *_data, uint16_t length, uint8_t polynomial = 0x31);
};

/// 全局串口数据单例对象
inline SerialData serialdata;
