## 时间同步头
# Header header

## 自瞄发给电控的target_mode //0跟随 1不跟随  暂时用不到
uint8 target_mode

## 自瞄发给电控的shoot_mode //0：不发射 1：发射
uint8 shoot_mode

## 自瞄目前的target编号
uint8 target

## 自瞄射击阈值内的敌人数量（误识别成同号装甲板会重复计算）  
uint8 shootable_num

# 0-英雄 1-工程 2-3号步兵 3-4号步兵 4-前哨 5-基地 6-哨兵 
#RMUL里面无1、3、4、5
## 自瞄目标存在状态vector(0-6，固定7位长，vector中索引位置与机器人编号相同，值为自瞄识别出的该编号目标个数)
uint8[] target_vector

## 自瞄目标距离vector(0-6，固定7位长，vector中索引位置与机器人编号相同，值为自瞄识别出的该编号目标个数)
float64[] distance_vector

## 自瞄目标极坐标极径vector(0-6，固定7位长，vector中索引位置与机器人编号相同，值为自瞄识别出的该编号目标个数)
float64[] polar_r_vector

## 自瞄目标极坐标极角vector(0-6，固定7位长，vector中索引位置与机器人编号相同，值为自瞄识别出的该编号目标个数)
float64[] polar_angle_vector #相对陀螺仪坐标系的角度

## 感知目标存在状态vector(0-6，固定7位长，vector中索引位置与机器人编号相同，值为自瞄识别出的该编号目标个数)
uint8[] detector_target_vector

## 感知目标距离vector(0-6，固定7位长，vector中索引位置与机器人编号相同，自瞄未识别出目标时应置为0)
float64[] detector_distance_vector

## 感知目标极坐标极径vector(0-6，固定7位长，vector中索引位置与机器人编号相同，感知未识别出目标时应置为0)
float64[] detector_polar_r_vector

## 感知目标极坐标极角vector(0-6，固定7位长，vector中索引位置与机器人编号相同，感知未识别出目标时应置为0)
float64[] detector_polar_angle_vector