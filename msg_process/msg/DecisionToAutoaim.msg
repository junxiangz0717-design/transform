## 决策指定射击模式（0命中优先|1猛灌）
uint8 decision_shoot_mode

## 决策希望优先击打的目标（不是必须击打的目标，当该目标丢失在自瞄视野内时，自瞄应该击打其他目标而不是不发射）
uint8 target

## 决策指定击打的颜色，0蓝2红（决策启用，自瞄未使用该标志位）
uint8 color

## 敌方机器人可攻击状态（1不无敌，0无敌，vector位置索引与敌方机器人编号相同）
uint8[] approachable_state_vector

## 决策自身yaw角（决策坐标系下）
float32 decide_yaw

## 大小yaw差值
float32 delta_yaw


# 自瞄有默认击打顺序
# 需要通过target调整残血顺序 和 approachable_state_vector帮助自瞄判断是否可以击打