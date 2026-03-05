## 云台手的目标点x坐标
float32 goalx
## 云台手的目标点y坐标
float32 goaly

## 当前比赛剩余时间
int32 time

## 哨兵机器人的血量
uint16 hp_sentry
## 己方前哨站的血量
uint16 hp_outpost

## 颜色，0表示己方为红，1表示己方为蓝
uint8 color

## 测速模块朝向,单位：度。正北为 0 度
float32 gps_yaw ##枪管yaw，即小yaw

## 剩余金币数量
uint16 money

## 己方英雄机器人位置 x 轴坐标，单位：m
float32 our_hero_x
## 己方英雄机器人位置 y 轴坐标，单位：m
float32 our_hero_y
## 己方步兵机器人位置 x 轴坐标，单位：m
float32 our_foot_3_x
## 己方步兵机器人位置 y 轴坐标，单位：m
float32 our_foot_3_y
## 己方步兵机器人位置 x 轴坐标，单位：m
float32 our_foot_4_x
## 己方步兵机器人位置 y 轴坐标，单位：m
float32 our_foot_4_y


## 我方英雄机器人血量。若该机器人未上场或者被罚下，则血量为 0
uint16 our_hero_hp
## 我方工程机器人血量
uint16 our_engineer_hp
## 我方3号步兵机器人血量
uint16 our_foot_3_hp
## 我方4号步兵机器人血量
uint16 our_foot_4_hp


## 敌方英雄机器人血量
uint16 enemy_hero_hp
## 敌方工程机器人血量
uint16 enemy_engineer_hp
## 敌方3号步兵机器人血量
uint16 enemy_foot_3_hp
## 敌方4号步兵机器人血量
uint16 enemy_foot_4_hp
## 敌方哨兵机器人血量
uint16 enemy_sentry_hp

## 己方基地的血量
uint16 hp_base
## 敌方基地的血量
uint16 hp_enemy_base
## 敌方前哨站的血量
uint16 hp_enemy_outpost

## 己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
uint8 add_area_state
## 中心增益点的占领情况，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领。（仅 RMUL 适用）
uint8 buff_area_state
## 己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数）
uint8 base_defence
## 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
uint8 add_hp_buff
## 机器人防御增益
uint8 defence_buff

## 17mm 弹丸允许发弹量
uint16 ammo

## RFID:己方补血点（检测到任一均视为激活）（小写是消息要求）
uint32 rfid
uint8 is_in_add_area
uint8 is_in_fort ##是否在堡垒区
## RFID:中心增益点（仅 RMUL 适用）(0没有 1我方 2对方 3双方)
uint8 is_center_area

## RFID:哨兵巡逻区（仅 RMUC 适用）
float32 chassis_power

## 裁判系统发的比赛阶段
uint8 game_period

## 比赛风格标志位，目前为0保守，1进击，2巡逻，3普通
uint8 style

## 电控算出来的枪口热量
uint16 heat_1

## 电控测出无法发弹
uint8 is_no_ammo

## 电控重启建图标志位
uint8 restart_mapping

float32 delta_yaw ##大小yaw差角

## 能量机关
uint8 rune 

uint8 remain_energy

## 雷达标定信息
float32[] radar_data
## float32 radar_enemy_hero_x
## float32 radar_enemy_hero_y
## float32 radar_enemy_engineer_x
## float32 radar_enemy_engineer_y
## float32 radar_enemy_foot_3_x
## float32 radar_enemy_foot_3_y
## float32 radar_enemy_foot_4_x
## float32 radar_enemy_foot_4_y
## float32 radar_enemy_sentry_x
## float32 radar_enemy_sentry_y


## 除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为 0，在哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人成功兑换的允许发弹量值
uint16 success_home_buy_ammo
## 哨兵机器人成功远程兑换允许发弹量的次数，开局为 0，在哨兵机器人成功远程兑换允许发弹量后，该值将变为哨兵机器人成功远程兑换允许发弹量的次数
uint8 success_remote_buy_ammo_times
## 哨兵机器人成功远程兑换血量的次数，开局为 0，在哨兵机器人成功远程兑换血量后，该值将变为哨兵机器人成功远程兑换血量的次数
uint8 success_remote_buy_hp_times
## 哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为1，否则为 0
uint8 can_free_respawn
## 哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时值为1，否则为 0
uint8 can_buy_respawn
## 哨兵机器人当前若兑换立即复活需要花费的金币数
uint16 buy_respawn_money

## 哨兵当前是否处于脱战状态，处于脱战状态时为 1，否则为 0
uint8 is_disengaged
## 队伍 17mm 允许发弹量的剩余可兑换数(RMUL暂时不用)
## uint16 team_residual_ammo

float32 vx
float32 vy

## 电控信息
float32 x_data
float32 y_data
float32 rotation_state
float32 temp_x
float32 temp_y
float32 temp_theta
float32 speed_x
float32 speed_y
float32 speed_z
uint8 wheel_state1
uint8 wheel_state2
uint8 wheel_state3
uint8 wheel_state4

