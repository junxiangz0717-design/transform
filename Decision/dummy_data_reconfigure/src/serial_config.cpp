#include "dummy_serial.hpp"

DummySerial::DummySerial() 
    : Node("dummy_serial")
{
    // 创建发布器
    serial_receive_data_pub_ = this->create_publisher<msg_process::msg::ReceiveData>(
        "/serial_receive_data", 10);
    pos_data_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/state_estimation", 10);

    // 声明所有参数（为枚举类型添加描述）
    this->declare_parameter("stop", params_.stop);
    this->declare_parameter("goal_x", params_.goal_x);
    this->declare_parameter("goal_y", params_.goal_y);
    this->declare_parameter("time", params_.time);
    this->declare_parameter("hp_sentry", params_.hp_sentry);
    this->declare_parameter("defence_buff", params_.defence_buff);
    this->declare_parameter("RFID_PATROL", params_.RFID_PATROL);
    this->declare_parameter("RFID_TREATMENT", params_.RFID_TREATMENT);
    this->declare_parameter("is_no_ammo", params_.is_no_ammo);
    this->declare_parameter("ammo", params_.ammo);
    this->declare_parameter("restart_decision_game", params_.restart_decision_game);
    this->declare_parameter("defence_base", params_.defence_base);
    this->declare_parameter("hp_base", params_.hp_base);
    this->declare_parameter("hp_outpost", params_.hp_outpost);
    this->declare_parameter("our_hero_hp", params_.our_hero_hp);
    this->declare_parameter("our_engineer_hp", params_.our_engineer_hp);
    this->declare_parameter("our_foot_3_hp", params_.our_foot_3_hp);
    this->declare_parameter("our_foot_4_hp", params_.our_foot_4_hp);
    this->declare_parameter("our_hero_x", params_.our_hero_x);
    this->declare_parameter("our_hero_y", params_.our_hero_y);
    this->declare_parameter("our_foot_3_x", params_.our_foot_3_x);
    this->declare_parameter("our_foot_3_y", params_.our_foot_3_y);
    this->declare_parameter("our_foot_4_x", params_.our_foot_4_x);
    this->declare_parameter("our_foot_4_y", params_.our_foot_4_y);
    this->declare_parameter("cur_x", params_.cur_x);
    this->declare_parameter("cur_y", params_.cur_y);
    this->declare_parameter("cur_yaw", params_.cur_yaw);
    this->declare_parameter("enemy_hero_hp", params_.enemy_hero_hp);
    this->declare_parameter("enemy_engineer_hp", params_.enemy_engineer_hp);
    this->declare_parameter("enemy_foot_3_hp", params_.enemy_foot_3_hp);
    this->declare_parameter("enemy_foot_4_hp", params_.enemy_foot_4_hp);
    this->declare_parameter("enemy_sentry_hp", params_.enemy_sentry_hp);
    this->declare_parameter("enemy_outpost_hp", params_.enemy_outpost_hp);
    this->declare_parameter("enemy_base_hp", params_.enemy_base_hp);
    this->declare_parameter("success_home_buy_ammo", params_.success_home_buy_ammo);
    this->declare_parameter("success_remote_buy_ammo_times", params_.success_remote_buy_ammo_times);
    this->declare_parameter("success_remote_buy_hp_times", params_.success_remote_buy_hp_times);
    this->declare_parameter("can_free_respawn", params_.can_free_respawn);
    this->declare_parameter("can_buy_respawn", params_.can_buy_respawn);
    this->declare_parameter("buy_respawn_money", params_.buy_respawn_money);
    this->declare_parameter("is_disengaged", params_.is_disengaged);
    this->declare_parameter("team_residual_ammo", params_.team_residual_ammo);
    this->declare_parameter("is_center_area", params_.is_center_area);
    
    // 枚举参数声明
    this->declare_parameter("color", static_cast<int>(params_.color), 
        create_enum_descriptor("己方颜色", {
            {0, "红色"},
            {1, "蓝色"}
        }));
    
    this->declare_parameter("game_period", static_cast<int>(params_.game_period),
        create_enum_descriptor("比赛阶段", {
            {0, "未开始"},
            {1, "准备中"},
            {2, "15秒裁判系统自检"},
            {3, "5秒倒计时"},
            {4, "比赛中"},
            {5, "比赛结算中"}
        }));
    
    this->declare_parameter("style", static_cast<int>(params_.style),
        create_enum_descriptor("比赛风格", {
            {0, "保守"},
            {1, "进击"},
            {2, "巡逻"},
            {3, "普通"},
            {4, "适应性训练4"},
            {5, "适应性训练5"}
        }));
    
    this->declare_parameter("dart_target", static_cast<int>(params_.dart_target),
        create_enum_descriptor("飞镖目标", {
            {0, "前哨站"},
            {1, "基地固定目标"},
            {2, "基地随机目标"}
        }));

    // 设置参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&DummySerial::param_callback, this, std::placeholders::_1));

    // 倒计时定时器：每秒递减时间（仅在比赛阶段4生效）  26新加
    countdown_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            // 只有在比赛中(game_period=4)且时间大于0时才倒计时
            if (params_.game_period == GamePeriod::GAMING && params_.time > 0) {
                params_.time--;
                // 同步更新 ROS 参数
                this->set_parameter(rclcpp::Parameter("time", params_.time));
                publish_data();
                RCLCPP_INFO(this->get_logger(), "比赛剩余时间: %d 秒", params_.time);
            }
        });

    RCLCPP_INFO(this->get_logger(), "serial 节点仿真启动");
}

rcl_interfaces::msg::ParameterDescriptor DummySerial::create_enum_descriptor(
    const std::string& description, 
    const std::vector<std::pair<int, std::string>>& enum_values)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.additional_constraints = "可选值: ";
    
    for (const auto& [value, name] : enum_values) {
        descriptor.additional_constraints += std::to_string(value) + "(" + name + "), ";
    }
    
    // 移除最后的逗号和空格
    if (!descriptor.additional_constraints.empty()) {
        descriptor.additional_constraints.pop_back();
        descriptor.additional_constraints.pop_back();
    }
    
    // 设置整数范围
    descriptor.integer_range.resize(1);
    if (!enum_values.empty()) {
        int min_val = enum_values.front().first;
        int max_val = enum_values.back().first;
        for (const auto& [value, name] : enum_values) {
            if (value < min_val) min_val = value;
            if (value > max_val) max_val = value;
        }
        descriptor.integer_range[0].from_value = min_val;
        descriptor.integer_range[0].to_value = max_val;
        descriptor.integer_range[0].step = 1;
    }
    
    return descriptor;
}

rcl_interfaces::msg::SetParametersResult DummySerial::param_callback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
        const std::string & name = param.get_name();
        
        try {
            // 更新参数值
            if (name == "stop") params_.stop = param.as_bool();
            else if (name == "goal_x") params_.goal_x = param.as_double();
            else if (name == "goal_y") params_.goal_y = param.as_double();
            else if (name == "time") params_.time = param.as_int();
            else if (name == "hp_sentry") params_.hp_sentry = param.as_int();
            else if (name == "defence_buff") params_.defence_buff = param.as_int();
            else if (name == "RFID_PATROL") params_.RFID_PATROL = param.as_bool();
            else if (name == "RFID_TREATMENT") params_.RFID_TREATMENT = param.as_bool();
            else if (name == "is_no_ammo") params_.is_no_ammo = param.as_bool();
            else if (name == "ammo") params_.ammo = param.as_int();
            else if (name == "restart_decision_game") params_.restart_decision_game = param.as_int();
            else if (name == "defence_base") params_.defence_base = param.as_int();
            else if (name == "hp_base") params_.hp_base = param.as_int();
            else if (name == "hp_outpost") params_.hp_outpost = param.as_int();
            else if (name == "our_hero_hp") params_.our_hero_hp = param.as_int();
            else if (name == "our_engineer_hp") params_.our_engineer_hp = param.as_int();
            else if (name == "our_foot_3_hp") params_.our_foot_3_hp = param.as_int();
            else if (name == "our_foot_4_hp") params_.our_foot_4_hp = param.as_int();
            else if (name == "our_hero_x") params_.our_hero_x = param.as_double();
            else if (name == "our_hero_y") params_.our_hero_y = param.as_double();
            else if (name == "our_foot_3_x") params_.our_foot_3_x = param.as_double();
            else if (name == "our_foot_3_y") params_.our_foot_3_y = param.as_double();
            else if (name == "our_foot_4_x") params_.our_foot_4_x = param.as_double();
            else if (name == "our_foot_4_y") params_.our_foot_4_y = param.as_double();
            else if (name == "cur_x") params_.cur_x = param.as_double();
            else if (name == "cur_y") params_.cur_y = param.as_double();
            else if (name == "cur_yaw") params_.cur_yaw = param.as_double();
            else if (name == "enemy_hero_hp") params_.enemy_hero_hp = param.as_int();
            else if (name == "enemy_engineer_hp") params_.enemy_engineer_hp = param.as_int();
            else if (name == "enemy_foot_3_hp") params_.enemy_foot_3_hp = param.as_int();
            else if (name == "enemy_foot_4_hp") params_.enemy_foot_4_hp = param.as_int();
            else if (name == "enemy_sentry_hp") params_.enemy_sentry_hp = param.as_int();
            else if (name == "enemy_outpost_hp") params_.enemy_outpost_hp = param.as_int();
            else if (name == "enemy_base_hp") params_.enemy_base_hp = param.as_int();
            else if (name == "success_home_buy_ammo") params_.success_home_buy_ammo = param.as_int();
            else if (name == "success_remote_buy_ammo_times") params_.success_remote_buy_ammo_times = param.as_int();
            else if (name == "success_remote_buy_hp_times") params_.success_remote_buy_hp_times = param.as_int();
            else if (name == "can_free_respawn") params_.can_free_respawn = param.as_bool();
            else if (name == "can_buy_respawn") params_.can_buy_respawn = param.as_bool();
            else if (name == "buy_respawn_money") params_.buy_respawn_money = param.as_int();
            else if (name == "is_disengaged") params_.is_disengaged = param.as_bool();
            else if (name == "team_residual_ammo") params_.team_residual_ammo = param.as_int();
            else if (name == "is_center_area") params_.is_center_area = param.as_int();
            
            // 枚举参数处理
            else if (name == "color") {
                int color_val = param.as_int();
                if (color_val == 0) params_.color = Color::RED;
                else if (color_val == 1) params_.color = Color::BLUE;
                else {
                    result.successful = false;
                    result.reason = "color must be 0(red) or 1(blue)";
                }
            }
            else if (name == "game_period") {
                int period_val = param.as_int();
                if (period_val >= 0 && period_val <= 5) {
                    params_.game_period = static_cast<GamePeriod>(period_val);
                } else {
                    result.successful = false;
                    result.reason = "game_period must be between 0-5";
                }
            }
            else if (name == "style") {
                int style_val = param.as_int();
                if (style_val >= 0 && style_val <= 5) {
                    params_.style = static_cast<Style>(style_val);
                } else {
                    result.successful = false;
                    result.reason = "style must be between 0-5";
                }
            }
            else if (name == "dart_target") {
                int dart_val = param.as_int();
                if (dart_val >= 0 && dart_val <= 2) {
                    params_.dart_target = static_cast<DartTarget>(dart_val);
                } else {
                    result.successful = false;
                    result.reason = "dart_target must be between 0-2";
                }
            }
        } catch (const std::exception& e) {
            result.successful = false;
            result.reason = std::string("错误: ") + e.what();
        }
    }

    if (result.successful) {
        // 发布更新后的数据
        publish_data();

        // 打印日志
        RCLCPP_INFO(this->get_logger(), 
                    "参数已更新，当前参数为：\n"
                    "1.云台手给点坐标：(%.2f, %.2f)\n"
                    "2.比赛剩余时间：%d\n"
                    "3.哨兵血量：%d\n"
                    "4.己方颜色：%d（红0/蓝1）\n"
                    "5.风格：%d\n"
                    "6.中心区占领情况：%d",
                    params_.goal_x, params_.goal_y,
                    params_.time,
                    params_.hp_sentry,
                    static_cast<int>(params_.color),
                    static_cast<int>(params_.style),
                    static_cast<int>(params_.is_center_area));
    }

    return result;
}

void DummySerial::publish_data()
{
    // 发布 serial_receive_data
    auto serial_data = msg_process::msg::ReceiveData();
    serial_data.goalx = params_.goal_x;
    serial_data.goaly = params_.goal_y;
    serial_data.time = params_.time;
    serial_data.hp_sentry = static_cast<uint16_t>(params_.hp_sentry);
    serial_data.defence_buff = static_cast<uint8_t>(params_.defence_buff);
    serial_data.color = static_cast<uint8_t>(params_.color);
    serial_data.style = static_cast<uint8_t>(params_.style);
    serial_data.is_in_add_area = static_cast<uint8_t>(params_.RFID_TREATMENT);
    serial_data.base_defence = static_cast<uint16_t>(params_.defence_base);
    serial_data.hp_base = static_cast<uint16_t>(params_.hp_base);
    serial_data.hp_outpost = static_cast<uint16_t>(params_.hp_outpost);
    serial_data.our_hero_hp = static_cast<uint16_t>(params_.our_hero_hp);
    serial_data.our_engineer_hp = static_cast<uint16_t>(params_.our_engineer_hp);
    serial_data.our_foot_3_hp = static_cast<uint16_t>(params_.our_foot_3_hp);
    serial_data.our_foot_4_hp = static_cast<uint16_t>(params_.our_foot_4_hp);
    serial_data.our_hero_x = static_cast<float>(params_.our_hero_x);
    serial_data.our_hero_y = static_cast<float>(params_.our_hero_y);
    serial_data.our_foot_3_x = static_cast<float>(params_.our_foot_3_x);
    serial_data.our_foot_3_y = static_cast<float>(params_.our_foot_3_y);
    serial_data.our_foot_4_x = static_cast<float>(params_.our_foot_4_x);
    serial_data.our_foot_4_y = static_cast<float>(params_.our_foot_4_y);
    serial_data.enemy_hero_hp = static_cast<uint16_t>(params_.enemy_hero_hp);
    serial_data.enemy_engineer_hp = static_cast<uint16_t>(params_.enemy_engineer_hp);
    serial_data.enemy_foot_3_hp = static_cast<uint16_t>(params_.enemy_foot_3_hp);
    serial_data.enemy_foot_4_hp = static_cast<uint16_t>(params_.enemy_foot_4_hp);
    serial_data.hp_enemy_outpost = static_cast<uint16_t>(params_.enemy_outpost_hp);
    serial_data.hp_enemy_base = static_cast<uint16_t>(params_.enemy_base_hp);
    serial_data.enemy_sentry_hp = static_cast<uint16_t>(params_.enemy_sentry_hp);
    serial_data.game_period = static_cast<uint8_t>(params_.game_period);
    serial_data.is_no_ammo = static_cast<uint8_t>(params_.is_no_ammo);
    serial_data.ammo = static_cast<uint16_t>(params_.ammo);
    serial_data.restart_decision_game = static_cast<uint8_t>(params_.restart_decision_game);
    serial_data.success_home_buy_ammo = static_cast<uint16_t>(params_.success_home_buy_ammo);
    serial_data.success_remote_buy_ammo_times = static_cast<uint8_t>(params_.success_remote_buy_ammo_times);
    serial_data.success_remote_buy_hp_times = static_cast<uint8_t>(params_.success_remote_buy_hp_times);
    serial_data.can_free_respawn = static_cast<uint8_t>(params_.can_free_respawn);
    serial_data.can_buy_respawn = static_cast<uint8_t>(params_.can_buy_respawn);
    serial_data.buy_respawn_money = static_cast<uint16_t>(params_.buy_respawn_money);
    serial_data.is_disengaged = static_cast<uint8_t>(params_.is_disengaged);
    // rmul
    serial_data.is_center_area = static_cast<uint8_t>(params_.is_center_area);
    
    
    // 初始化雷达数据
    serial_data.radar_data.assign(10, 0.0);

    serial_receive_data_pub_->publish(serial_data);

    // 发布 odometry 数据
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.pose.pose.position.x = static_cast<float>(params_.cur_x);
    odom_msg.pose.pose.position.y = static_cast<float>(params_.cur_y);
    
    // 设置消息的header
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    // 将yaw角转换为四元数
    tf2::Quaternion quat;
    quat.setRPY(0, 0, params_.cur_yaw);
    odom_msg.pose.pose.orientation = tf2::toMsg(quat);
    
    pos_data_pub_->publish(odom_msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DummySerial>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}