//
// Created by zh on 24-3-12.
//
#pragma once
#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

class Process_Bar
{
private:
    int all_block_num = 1000;
    std::string prompt;
    BT::Tree* tree_ = nullptr;

public:
    explicit Process_Bar(std::string_view prompt = ""): prompt(prompt){};

    explicit Process_Bar(BT::Tree& tree, std::string_view prompt = ""): tree_(&tree), prompt(prompt){};

    Process_Bar(BT::Tree& tree, int million_sec, std::string_view prompt = ""): Process_Bar(tree, prompt)
    {
        this->operator()(million_sec);
    }

    void operator() (int million_sec)
    {
        all_block_num = million_sec;

        std::cout << std::endl << prompt << "\x1b[33m" << std::endl;
        for (int i = 0; i<all_block_num/10; i++)
        {
            printf("\r[%.2lfs | %.2lf%%]:", float(all_block_num)/1000 ,i*100.0 / (all_block_num - 1)*10);
            int show_num = i * 200 / all_block_num;
            for (int j = 1; j <= show_num; j++)
            {
                std::cout << "█";
            }
            if (tree_ == nullptr)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            else
                tree_->sleep(std::chrono::milliseconds(10));
        }

        printf("\r\x1b[32m[%.2lfs | 100.00%%]", float(all_block_num)/1000);
        for (int j = 1; j <= 20; j++)
        {
            std::cout << "█";
        }
        std::cout << "->完成！" << "\x1b[0m" << std::endl << std::endl;
    }
};