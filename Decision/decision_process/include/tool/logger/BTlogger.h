//重写日志器，忽略prev_status为IDLE的状态输出
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

// ANSI 转义序列定义不同颜色
const std::string COLOR_RESET = "\x1b[0m";
const std::string COLOR_GREEN = "\x1b[92m";
const std::string COLOR_RED = "\x1b[91m";
const std::string COLOR_YELLOW = "\x1b[33m";
const std::string COLOR_BLUE = "\x1b[94m";

// 自定义日志记录器类，继承自 StatusChangeLogger
class CustomLogger : public BT::StatusChangeLogger
{
public:
    CustomLogger(const BT::Tree& tree) : BT::StatusChangeLogger(tree.rootNode()) {}

    // 重写 callback 函数
    void callback(BT::Duration timestamp, const BT::TreeNode& node,
                  BT::NodeStatus prev_status, BT::NodeStatus status) override
    {
        if(node.name() !="ForceSuccess" ) 
        {
            if( node.name() !="MultiAsyncReactiveSequence")
            {
                    // 过滤掉状态变为 IDLE 的消息
                if (status != BT::NodeStatus::IDLE)
                {
                    std::string color;
                    switch (status)
                    {
                    case BT::NodeStatus::SUCCESS:
                        color = COLOR_GREEN;
                        break;
                    case BT::NodeStatus::FAILURE:
                        color = COLOR_RED;
                        break;
                    case BT::NodeStatus::RUNNING:
                        color = COLOR_YELLOW;
                        break;
                    default:
                        color = COLOR_RESET;
                        break;
                    }
                    std::string prev_color;
                    switch (prev_status)
                    {
                    case BT::NodeStatus::SUCCESS:
                        prev_color = COLOR_GREEN;
                        break;
                    case BT::NodeStatus::FAILURE:
                        prev_color = COLOR_RED;
                        break;
                    case BT::NodeStatus::RUNNING:
                        prev_color = COLOR_YELLOW;
                        break;
                    case BT::NodeStatus::IDLE:
                        prev_color = COLOR_BLUE;
                        break;
                    default:
                        prev_color = COLOR_RESET;
                        break;
                    }
                    std::cout << "[" << timestamp.count() << "] "
                            << node.name() << ": "
                            << prev_color << BT::toStr(prev_status) << COLOR_RESET<<" -> "
                            << color << BT::toStr(status) << COLOR_RESET << std::endl;
                }
            }
            
        }

       
    }

    // 重写 flush 函数
    void flush() override {}
};