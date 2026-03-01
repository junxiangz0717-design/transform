//
// Created by zh on 24-3-4.
//
# pragma once

#include "behaviortree_cpp/controls/reactive_sequence.h"

using namespace BT;

class MultiAsyncReactiveSequence: public ReactiveSequence
{
public:
    MultiAsyncReactiveSequence(const std::string& name) : ReactiveSequence(name)
    {
        EnableException(false);
    }
};