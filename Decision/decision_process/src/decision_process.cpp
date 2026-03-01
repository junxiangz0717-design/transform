
#include "rclcpp/rclcpp.hpp"

#include "sub_pub/sub.h"
#include "sub_pub/pub.h"
#include "tree/decision_behaviortree.h"
//工具
#include "tool/time/time_measure.h"
#include "tool/logger/logger.h"
//数据
#include "data/decision_data.h"
#include "data/enum.h"


//启动入口

int main(int argc, char* argv[])
{

    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("decision_process");

    PointManager::registerElement("自身位置", DD.self_pos);
    DD.targets.register_to_manager();

    {
        // 把与 ROS 资源绑定的对象放在局部作用域中
        auto data_suber = std::make_shared<DataSubscriber>(node);
        rclcpp::sleep_for(std::chrono::milliseconds(200));

        // 对应 ROS1 的 mes_puber = MessagePublisher(nh);
        mes_puber = std::make_shared<MessagePublisher>(node);

        rclcpp::sleep_for(std::chrono::milliseconds(200));

        decision_behaviortree behaviortree(DD.config_HZ);

        /*让主线程作为 node 的 executor，处理回调；当 spin 返回时我们会退出此作用域，
        触发 data_suber、behaviortree 等对象的析构，保证它们在 rclcpp::shutdown() 之前被销毁*/
        rclcpp::spin(node);

        // 显式释放全局 publisher 对象，确保其 publisher 在 rcl shutdown 之前被删除。
        mes_puber.reset();
        // 作用域结束，data_suber 和 behaviortree 的析构函数会自动运行（tree 线程会 join）。
    }

    // 在所有与 node 绑定的对象都已被安全销毁后，再执行 shutdown
    rclcpp::shutdown();
    return 0;


}
