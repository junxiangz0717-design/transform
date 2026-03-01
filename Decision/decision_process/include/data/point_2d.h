//
// Created by zh on 24-3-29.
//
#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include <deque>

using namespace BT;

class point_2d
{
private:
    double x_;
    double y_;
public:
    point_2d() = default;
    point_2d(double x, double y): x_(x), y_(y){}

    void operator() (double x, double y)
    {
        x_  = x;
        y_  = y;
    }

    void operator() (point_2d pos)
    {
        x_ = pos.getX();
        y_ = pos.getY();
    }

    inline double getX() const{ return x_;}
    inline double getY() const{ return y_;}
};

namespace BT
{
    template <> inline
    SharedQueue<point_2d> convertFromString<SharedQueue<point_2d>>(StringView str)
    {
        auto parts = splitString(str, ';');
        SharedQueue<point_2d> output = std::make_shared<std::deque<point_2d>>();
        for (const StringView& part : parts)
        {
            double x, y;
            auto x_y_vec = splitString(part, ',');
            x = convertFromString<double>(x_y_vec[0]);
            y = convertFromString<double>(x_y_vec[1]);
            output->push_back(point_2d{x, y});
        }
        return output;
    }
}
