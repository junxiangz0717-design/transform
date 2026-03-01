//树节点运行日志
#pragma once

#include <ctime>
#include <iomanip>
#include <fstream>

#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "data/decision_data.h"
#include "path/decision_package_path.h"

using namespace std;
using namespace BT;


enum ZhLoggerMode
{
    NOW,
    START_TIME,
    LOCAL_START_TIME
};


class ZhCoutLogger: public StatusChangeLogger
{
private:
    Tree* tree_;
    ZhLoggerMode logger_mode_;
    ofstream logFile;

public:
    ZhCoutLogger(const Tree& tree, const ZhLoggerMode& loggerMode);

    ~ZhCoutLogger() override;

    //Duration:  std::chrono下高精度时间间隔类
    void callback(Duration timestamp, const TreeNode& node,
                                 NodeStatus prev_status, NodeStatus status) override;

    void flush() override;//强制刷新输出缓冲区
};



