#include "logger.h"

#include <filesystem>

ZhCoutLogger::ZhCoutLogger(const Tree &tree, const ZhLoggerMode &loggerMode) :
        StatusChangeLogger(tree.rootNode()), logger_mode_(loggerMode),
        logFile("")
        {
            // Ensure the node log parent directory exists, then open the file
            try {
                std::filesystem::path p(DECISION_PACKAGE_PATH_NODE);
                if (!p.parent_path().empty()) {
                    std::filesystem::create_directories(p.parent_path());
                }
                // If DECISION_PACKAGE_PATH_NODE is a directory-like path, open a file named "node_log"
                logFile.open(DECISION_PACKAGE_PATH_NODE, ios::app);
            } catch (const std::exception &e) {
                std::cerr << "创建节点日志目录或打开文件时出错: " << e.what() << std::endl;
            }
            if(!logFile.is_open()) {
                cout<<"文件打开失败"<<endl;
            }
        }

void ZhCoutLogger::callback(Duration timestamp, const TreeNode &node, NodeStatus prev_status, NodeStatus status) {
    constexpr const char* whitespaces = "                         ";
    constexpr const size_t ws_count = 25;

    if (node.name()=="决策循环进入点" && status==NodeStatus::SUCCESS)
    {
        cout << "\x1b[34m";
        for (int i = 0; i < 40; ++i) {
            cout << "-";
        }
        cout << "\x1b[0m" << endl;
        //新增文件功能
        logFile << "\x1b[34m";
        for (int i = 0; i < 40; ++i) {
            logFile << "-";
        }
        logFile << "\x1b[0m" << endl;
    }

    if (logger_mode_ == START_TIME)
    {
        printf("[%.3ds]: %s%s %s -> %s\n", DD.start_time, node.name().c_str(),
               &whitespaces[std::min(ws_count, node.name().size())],
               toStr(prev_status, true).c_str(), toStr(status, true).c_str());
        logFile << "[%.3ds]: " << node.name().c_str()
                << &whitespaces[std::min(ws_count, node.name().size())]
                << toStr(prev_status, true).c_str()
                << " -> " << toStr(status, true).c_str() << endl;
    }

    else {
        using namespace std::chrono;
        double since_epoch = duration<double>(timestamp).count();

        if (logger_mode_ == NOW)
        {
            // 将纪元时间timestamp转换为小时：分：秒.毫秒形式输出
            auto hours = (std::chrono::duration_cast<std::chrono::hours>(timestamp).count() + 8) % 24;
            auto minutes = std::chrono::duration_cast<std::chrono::minutes>(timestamp % std::chrono::hours(1)).count();
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(timestamp % std::chrono::minutes(1)).count();
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp % std::chrono::seconds(1)).count();

            // 输出结果
            std::cout << "["
                      << std::setw(2) << std::setfill('0') << hours << ":"
                      << std::setw(2) << std::setfill('0') << minutes << ":"
                      << std::setw(2) << std::setfill('0') << seconds << "."
                      << std::setw(3) << std::setfill('0') << milliseconds << "]: ";
            printf("%s%s %s -> %s\n", node.name().c_str(),
                   &whitespaces[std::min(ws_count, node.name().size())],
                   toStr(prev_status, true).c_str(), toStr(status, true).c_str());

            logFile << "["
                    << std::setw(2) << std::setfill('0') << hours << ":"
                    << std::setw(2) << std::setfill('0') << minutes << ":"
                    << std::setw(2) << std::setfill('0') << seconds << "."
                    << std::setw(3) << std::setfill('0') << milliseconds << "]: ";
            logFile << node.name().c_str()
                    << &whitespaces[std::min(ws_count, node.name().size())]
                    << toStr(prev_status, true).c_str()
                    << " -> " << toStr(status, true).c_str() << endl;
        }
        else if (logger_mode_ == LOCAL_START_TIME)
        {
            auto start_chrono = timestamp - DD.start_time_chrono;

            auto minutes = std::chrono::duration_cast<std::chrono::minutes>(start_chrono % std::chrono::hours(1)).count();
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(start_chrono % std::chrono::minutes(1)).count();
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(start_chrono % std::chrono::seconds(1)).count();

            // 输出结果
            std::cout << "["
                      << std::setw(2) << std::setfill('0') << minutes << ":"
                      << std::setw(2) << std::setfill('0') << seconds << "."
                      << std::setw(3) << std::setfill('0') << milliseconds << "]: ";
            printf("%s%s %s -> %s\n", node.name().c_str(),
                   &whitespaces[std::min(ws_count, node.name().size())],
                   toStr(prev_status, true).c_str(), toStr(status, true).c_str());

            logFile << "["
                    << std::setw(2) << std::setfill('0') << minutes << ":"
                    << std::setw(2) << std::setfill('0') << seconds << "."
                    << std::setw(3) << std::setfill('0') << milliseconds << "]: ";
            logFile << node.name().c_str()
                    << &whitespaces[std::min(ws_count, node.name().size())]
                    << toStr(prev_status, true).c_str()
                    << " -> " << toStr(status, true).c_str() << endl;

        }
    }
    std::cout << std::endl;
    logFile << endl;
}

ZhCoutLogger::~ZhCoutLogger() 
    {
        cout << "日志器掉线！" << endl;
        if (logFile.is_open()) 
        {
            logFile << "日志器掉线！" << endl;
            logFile.close();
        }
    }

void ZhCoutLogger::flush() {
    std::cout << std::flush;
    if (logFile.is_open()) 
    {
        logFile << std::flush;
    }
}
