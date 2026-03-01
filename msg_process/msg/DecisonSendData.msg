

# ##########
# #########


## 姿态
uint8 posture

uint8 az ##感知转头
## 是否由决策接管相对yaw角
## 置true时决策发的decision_yaw可以控制云台相对当前位置要转动的yaw
## 置false时不要使用decision_yaw，模拟云台相对yaw角被自瞄接管
uint8 is_decision_yaw  ##

# ## 控制云台相对当前位置要转动的yaw，正数代表逆时针旋转，负数反之
float32 decision_yaw_az ##用这个跟区分导航yaw


## 底盘小陀螺旋转标志位 0电控默认转（相当于决策没发）1开 2关
uint8 spin_mode

## 云台360°自转旋转标志位 0电控默认转（相当于决策没发）1-4开 5关
uint8 tripod_spin

# # ## 比赛风格标志位，目前为0保守，1进击，2巡逻
uint8 style
uint8 attack_rune

## 裁判系统哨兵自主决策数据包
## uint32 auto_decision_package
## 哨兵机器人是否确认复活 0:否 1:是
uint8 confirm_respawn
## 哨兵机器人是否确认兑换立即复活 0:否 1:是
uint8 buy_respawn
## 哨兵将要兑换的发弹量值，开局为 0，修改此值后，哨兵在补血点即可兑换允许发弹量，此值的变化需要单调递增，否则视为不合法
uint16 home_buy_ammo
## 哨兵远程兑换发弹量的请求次数，开局为 0，修改此值即可请求远程兑换发弹量，此值的变化需要单调递增且每次仅能增加 1，否则视为不合法
uint8 remote_buy_ammo_times
## 哨兵远程兑换血量的请求次数，开局为 0，修改此值即可请求远程兑换血量。此值的变化需要单调递增且每次仅能增加 1，否则视为不合法
uint8 remote_buy_hp_times

# ## pitch档位 1:前哨站（25—40）2:坡上打坡下（-25 — -10）3:挨打反击（-25—5）其他:全
uint8 pitch_mode

float32 pathplan_yaw#用这个控制大yaw
uint8 can_stop

uint8 detector_flag
## 注：决策有两套控制云台转动的方法，一个是让云台360°旋转，另一个是通过指定相对yaw角实现扇形搜索
## 决策在发decision_yaw时会把is_decision_yaw置true、tripod_spin置false
## 同理，在给tripod_spin置true时会把is_decision_yaw置false、decision_yaw置零

##牛魔wzc