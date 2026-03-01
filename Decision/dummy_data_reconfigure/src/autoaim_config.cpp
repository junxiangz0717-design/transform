#include "dummy_autoaim.hpp"
#include <map>

DummyAutoaim::DummyAutoaim() : Node("dummy_autoaim")
{
    autoaim_to_decision_pub_ = this->create_publisher<msg_process::msg::AutoaimToDecision>("/autoaim_to_decision", 10);

    // 创建参数描述映射
    std::map<std::string, std::string> param_descriptions = {
        {"stop", "停止参数发布"},
        {"target_mode", "目标状态"},
        {"target", "自瞄目标"},
        {"detector_target", "感知目标"},
        {"shootable_num", "打击范围内敌人数量"},
        // 自瞄开关描述
        {"auto_0", "自瞄英雄"},
        {"auto_1", "自瞄工程"},
        {"auto_2", "自瞄3号步兵"},
        {"auto_3", "自瞄4号步兵"},
        {"auto_4", "自瞄前哨"},
        {"auto_5", "自瞄基地"},
        {"auto_6", "自瞄哨兵"},
        // 自瞄距离描述
        {"auto_distance_0", "自瞄英雄距离"},
        {"auto_distance_1", "自瞄工程距离"},
        {"auto_distance_2", "自瞄3号步兵距离"},
        {"auto_distance_3", "自瞄4号步兵距离"},
        {"auto_distance_4", "自瞄前哨距离"},
        {"auto_distance_5", "自瞄基地距离"},
        {"auto_distance_6", "自瞄哨兵距离"},
        // 感知开关描述
        {"detector_0", "感知英雄"},
        {"detector_1", "感知工程"},
        {"detector_2", "感知3号步兵"},
        {"detector_3", "感知4号步兵"},
        {"detector_4", "感知前哨"},
        {"detector_5", "感知基地"},
        {"detector_6", "感知哨兵"},
        // 感知距离描述
        {"detector_distance_0", "感知英雄距离"},
        {"detector_distance_1", "感知工程距离"},
        {"detector_distance_2", "感知3号步兵距离"},
        {"detector_distance_3", "感知4号步兵距离"},
        {"detector_distance_4", "感知前哨距离"},
        {"detector_distance_5", "感知基地距离"},
        {"detector_distance_6", "感知哨兵距离"}};

    // 枚举参数
    this->declare_parameter("target", static_cast<int>(params_.target),
                            create_enum_descriptor("自瞄目标", {{0, "英雄"}, {1, "工程"}, {2, "3号步兵"}, {3, "4号步兵"}, {4, "前哨站"}, {5, "基地"}, {6, "哨兵"}}));

    this->declare_parameter("detector_target", static_cast<int>(params_.detector_target),
                            create_enum_descriptor("感知目标", {{0, "英雄"}, {1, "工程"}, {2, "3号步兵"}, {3, "4号步兵"}, {4, "前哨站"}, {5, "基地"}, {6, "哨兵"}}));

    // 布尔参数
    this->declare_parameter("stop", params_.stop, create_bool_descriptor(param_descriptions["stop"]));
    this->declare_parameter("target_mode", params_.target_mode, create_bool_descriptor(param_descriptions["target_mode"]));

    // 自瞄开关参数
    this->declare_parameter("auto_0", params_.auto_0, create_int_descriptor(param_descriptions["auto_0"], 0, 1));
    this->declare_parameter("auto_1", params_.auto_1, create_int_descriptor(param_descriptions["auto_1"], 0, 1));
    this->declare_parameter("auto_2", params_.auto_2, create_int_descriptor(param_descriptions["auto_2"], 0, 1));
    this->declare_parameter("auto_3", params_.auto_3, create_int_descriptor(param_descriptions["auto_3"], 0, 1));
    this->declare_parameter("auto_4", params_.auto_4, create_int_descriptor(param_descriptions["auto_4"], 0, 1));
    this->declare_parameter("auto_5", params_.auto_5, create_int_descriptor(param_descriptions["auto_5"], 0, 1));
    this->declare_parameter("auto_6", params_.auto_6, create_int_descriptor(param_descriptions["auto_6"], 0, 1));

    // 感知开关参数
    this->declare_parameter("detector_0", params_.detector_0, create_int_descriptor(param_descriptions["detector_0"], 0, 1));
    this->declare_parameter("detector_1", params_.detector_1, create_int_descriptor(param_descriptions["detector_1"], 0, 1));
    this->declare_parameter("detector_2", params_.detector_2, create_int_descriptor(param_descriptions["detector_2"], 0, 1));
    this->declare_parameter("detector_3", params_.detector_3, create_int_descriptor(param_descriptions["detector_3"], 0, 1));
    this->declare_parameter("detector_4", params_.detector_4, create_int_descriptor(param_descriptions["detector_4"], 0, 1));
    this->declare_parameter("detector_5", params_.detector_5, create_int_descriptor(param_descriptions["detector_5"], 0, 1));
    this->declare_parameter("detector_6", params_.detector_6, create_int_descriptor(param_descriptions["detector_6"], 0, 1));

    // 整数参数
    this->declare_parameter("shootable_num", params_.shootable_num, create_int_descriptor(param_descriptions["shootable_num"], 0, 7));

    // 自瞄距离参数
    this->declare_parameter("auto_distance_0", params_.auto_distance_0, create_double_descriptor(param_descriptions["auto_distance_0"], 1.0, 10.0));
    this->declare_parameter("auto_distance_1", params_.auto_distance_1, create_double_descriptor(param_descriptions["auto_distance_1"], 1.0, 10.0));
    this->declare_parameter("auto_distance_2", params_.auto_distance_2, create_double_descriptor(param_descriptions["auto_distance_2"], 1.0, 10.0));
    this->declare_parameter("auto_distance_3", params_.auto_distance_3, create_double_descriptor(param_descriptions["auto_distance_3"], 1.0, 10.0));
    this->declare_parameter("auto_distance_4", params_.auto_distance_4, create_double_descriptor(param_descriptions["auto_distance_4"], 1.0, 10.0));
    this->declare_parameter("auto_distance_5", params_.auto_distance_5, create_double_descriptor(param_descriptions["auto_distance_5"], 1.0, 10.0));
    this->declare_parameter("auto_distance_6", params_.auto_distance_6, create_double_descriptor(param_descriptions["auto_distance_6"], 1.0, 10.0));

    // 感知距离参数
    this->declare_parameter("detector_distance_0", params_.detector_distance_0, create_double_descriptor(param_descriptions["detector_distance_0"], 1.0, 10.0));
    this->declare_parameter("detector_distance_1", params_.detector_distance_1, create_double_descriptor(param_descriptions["detector_distance_1"], 1.0, 10.0));
    this->declare_parameter("detector_distance_2", params_.detector_distance_2, create_double_descriptor(param_descriptions["detector_distance_2"], 1.0, 10.0));
    this->declare_parameter("detector_distance_3", params_.detector_distance_3, create_double_descriptor(param_descriptions["detector_distance_3"], 1.0, 10.0));
    this->declare_parameter("detector_distance_4", params_.detector_distance_4, create_double_descriptor(param_descriptions["detector_distance_4"], 1.0, 10.0));
    this->declare_parameter("detector_distance_5", params_.detector_distance_5, create_double_descriptor(param_descriptions["detector_distance_5"], 1.0, 10.0));
    this->declare_parameter("detector_distance_6", params_.detector_distance_6, create_double_descriptor(param_descriptions["detector_distance_6"], 1.0, 10.0));

    // 设置参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&DummyAutoaim::param_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "自瞄仿真节点启动");
}

// 参数描述创建方法的实现
rcl_interfaces::msg::ParameterDescriptor DummyAutoaim::create_enum_descriptor(
    const std::string &description,
    const std::vector<std::pair<int, std::string>> &enum_values)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.additional_constraints = "可选值: ";

    for (const auto &[value, name] : enum_values)
    {
        descriptor.additional_constraints += std::to_string(value) + "(" + name + "), ";
    }

    if (!descriptor.additional_constraints.empty())
    {
        descriptor.additional_constraints.pop_back();
        descriptor.additional_constraints.pop_back();
    }

    descriptor.integer_range.resize(1);
    if (!enum_values.empty())
    {
        int min_val = enum_values.front().first;
        int max_val = enum_values.back().first;
        for (const auto &[value, name] : enum_values)
        {
            if (value < min_val)
                min_val = value;
            if (value > max_val)
                max_val = value;
        }
        descriptor.integer_range[0].from_value = min_val;
        descriptor.integer_range[0].to_value = max_val;
        descriptor.integer_range[0].step = 1;
    }

    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor DummyAutoaim::create_bool_descriptor(const std::string &description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor DummyAutoaim::create_int_descriptor(const std::string &description, int min, int max)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = min;
    descriptor.integer_range[0].to_value = max;
    descriptor.integer_range[0].step = 1;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor DummyAutoaim::create_double_descriptor(const std::string &description, double min, double max)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = min;
    descriptor.floating_point_range[0].to_value = max;
    descriptor.floating_point_range[0].step = 0.1;
    return descriptor;
}

rcl_interfaces::msg::SetParametersResult DummyAutoaim::param_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
        const std::string &name = param.get_name();

        try
        {
            // 更新参数值
            if (name == "stop")
                params_.stop = param.as_bool();
            else if (name == "target_mode")
                params_.target_mode = param.as_bool();
            else if (name == "shootable_num")
                params_.shootable_num = param.as_int();

            // 枚举参数处理
            else if (name == "target")
            {
                int target_val = param.as_int();
                if (target_val >= 0 && target_val <= 6)
                {
                    params_.target = static_cast<RobotType>(target_val);
                }
                else
                {
                    result.successful = false;
                    result.reason = "target must be between 0-6";
                }
            }
            else if (name == "detector_target")
            {
                int detector_val = param.as_int();
                if (detector_val >= 0 && detector_val <= 6)
                {
                    params_.detector_target = static_cast<RobotType>(detector_val);
                }
                else
                {
                    result.successful = false;
                    result.reason = "detector_target must be between 0-6";
                }
            }

            // 自瞄开关
            else if (name == "auto_0")
                params_.auto_0 = param.as_int();
            else if (name == "auto_1")
                params_.auto_1 = param.as_int();
            else if (name == "auto_2")
                params_.auto_2 = param.as_int();
            else if (name == "auto_3")
                params_.auto_3 = param.as_int();
            else if (name == "auto_4")
                params_.auto_4 = param.as_int();
            else if (name == "auto_5")
                params_.auto_5 = param.as_int();
            else if (name == "auto_6")
                params_.auto_6 = param.as_int();

            // 感知开关
            else if (name == "detector_0")
                params_.detector_0 = param.as_int();
            else if (name == "detector_1")
                params_.detector_1 = param.as_int();
            else if (name == "detector_2")
                params_.detector_2 = param.as_int();
            else if (name == "detector_3")
                params_.detector_3 = param.as_int();
            else if (name == "detector_4")
                params_.detector_4 = param.as_int();
            else if (name == "detector_5")
                params_.detector_5 = param.as_int();
            else if (name == "detector_6")
                params_.detector_6 = param.as_int();

            // 自瞄距离
            else if (name == "auto_distance_0")
                params_.auto_distance_0 = param.as_double();
            else if (name == "auto_distance_1")
                params_.auto_distance_1 = param.as_double();
            else if (name == "auto_distance_2")
                params_.auto_distance_2 = param.as_double();
            else if (name == "auto_distance_3")
                params_.auto_distance_3 = param.as_double();
            else if (name == "auto_distance_4")
                params_.auto_distance_4 = param.as_double();
            else if (name == "auto_distance_5")
                params_.auto_distance_5 = param.as_double();
            else if (name == "auto_distance_6")
                params_.auto_distance_6 = param.as_double();

            // 感知距离
            else if (name == "detector_distance_0")
                params_.detector_distance_0 = param.as_double();
            else if (name == "detector_distance_1")
                params_.detector_distance_1 = param.as_double();
            else if (name == "detector_distance_2")
                params_.detector_distance_2 = param.as_double();
            else if (name == "detector_distance_3")
                params_.detector_distance_3 = param.as_double();
            else if (name == "detector_distance_4")
                params_.detector_distance_4 = param.as_double();
            else if (name == "detector_distance_5")
                params_.detector_distance_5 = param.as_double();
            else if (name == "detector_distance_6")
                params_.detector_distance_6 = param.as_double();
        }
        catch (const std::exception &e)
        {
            result.successful = false;
            result.reason = std::string("错误: ") + e.what();
        }
    }

    if (result.successful)
    {
        // 发布更新后的数据
        publish_data();

        // 记录日志
        log_current_params();
    }

    return result;
}

void DummyAutoaim::publish_data()
{
    auto autoaim_data_ = msg_process::msg::AutoaimToDecision();

    autoaim_data_.target_vector.resize(7, 0);
    autoaim_data_.distance_vector.resize(7, 0.0);
    autoaim_data_.polar_r_vector.resize(7, 0.0);
    autoaim_data_.polar_angle_vector.resize(7, 0.0);
    autoaim_data_.detector_target_vector.resize(7, 0);
    autoaim_data_.detector_distance_vector.resize(7, 0.0);
    autoaim_data_.detector_polar_r_vector.resize(7, 0.0);
    autoaim_data_.detector_polar_angle_vector.resize(7, 0.0);

    // 设置基本参数
    autoaim_data_.target_mode = static_cast<uint8_t>(params_.target_mode);
    autoaim_data_.target = static_cast<uint8_t>(params_.target);
    autoaim_data_.shootable_num = static_cast<uint8_t>(params_.shootable_num);

    // 设置自瞄目标向量
    autoaim_data_.target_vector[0] = static_cast<uint8_t>(params_.auto_0);
    autoaim_data_.target_vector[1] = static_cast<uint8_t>(params_.auto_1);
    autoaim_data_.target_vector[2] = static_cast<uint8_t>(params_.auto_2);
    autoaim_data_.target_vector[3] = static_cast<uint8_t>(params_.auto_3);
    autoaim_data_.target_vector[4] = static_cast<uint8_t>(params_.auto_4);
    autoaim_data_.target_vector[5] = static_cast<uint8_t>(params_.auto_5);
    autoaim_data_.target_vector[6] = static_cast<uint8_t>(params_.auto_6);

    // 设置自瞄距离向量
    autoaim_data_.distance_vector[0] = static_cast<float>(params_.auto_distance_0);
    autoaim_data_.distance_vector[1] = static_cast<float>(params_.auto_distance_1);
    autoaim_data_.distance_vector[2] = static_cast<float>(params_.auto_distance_2);
    autoaim_data_.distance_vector[3] = static_cast<float>(params_.auto_distance_3);
    autoaim_data_.distance_vector[4] = static_cast<float>(params_.auto_distance_4);
    autoaim_data_.distance_vector[5] = static_cast<float>(params_.auto_distance_5);
    autoaim_data_.distance_vector[6] = static_cast<float>(params_.auto_distance_6);

    // 设置感知目标向量
    autoaim_data_.detector_target_vector[0] = static_cast<uint8_t>(params_.detector_0);
    autoaim_data_.detector_target_vector[1] = static_cast<uint8_t>(params_.detector_1);
    autoaim_data_.detector_target_vector[2] = static_cast<uint8_t>(params_.detector_2);
    autoaim_data_.detector_target_vector[3] = static_cast<uint8_t>(params_.detector_3);
    autoaim_data_.detector_target_vector[4] = static_cast<uint8_t>(params_.detector_4);
    autoaim_data_.detector_target_vector[5] = static_cast<uint8_t>(params_.detector_5);
    autoaim_data_.detector_target_vector[6] = static_cast<uint8_t>(params_.detector_6);

    // 设置感知距离向量
    autoaim_data_.detector_distance_vector[0] = static_cast<float>(params_.detector_distance_0);
    autoaim_data_.detector_distance_vector[1] = static_cast<float>(params_.detector_distance_1);
    autoaim_data_.detector_distance_vector[2] = static_cast<float>(params_.detector_distance_2);
    autoaim_data_.detector_distance_vector[3] = static_cast<float>(params_.detector_distance_3);
    autoaim_data_.detector_distance_vector[4] = static_cast<float>(params_.detector_distance_4);
    autoaim_data_.detector_distance_vector[5] = static_cast<float>(params_.detector_distance_5);
    autoaim_data_.detector_distance_vector[6] = static_cast<float>(params_.detector_distance_6);

    // 发布消息
    autoaim_to_decision_pub_->publish(autoaim_data_);
}

void DummyAutoaim::log_current_params()
{
    RCLCPP_INFO(this->get_logger(),
                "自瞄参数已更新:\n"
                "目标状态: %s\n"
                "自瞄目标: %d, 感知目标: %d\n"
                "打击范围内敌人: %d",
                params_.target_mode ? "开启" : "关闭",
                static_cast<int>(params_.target),
                static_cast<int>(params_.detector_target),
                params_.shootable_num);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DummyAutoaim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
