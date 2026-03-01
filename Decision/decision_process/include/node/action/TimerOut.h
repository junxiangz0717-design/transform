#pragma once

#include "behaviortree_cpp/action_node.h"
#include "tool/time/zh_timer.h"

using namespace std;
using namespace BT;

class TimerOut: public StatefulActionNode
{
private:
    zh_timer timeout;
    int temp;
    int type;
public:

    TimerOut(const string& name, const NodeConfig& config): StatefulActionNode(name, config)
    {}

    ~TimerOut() override = default;

    static PortsList providedPorts()
    {
        return
        {
            InputPort<int>("time_","超时时间"),
            InputPort<int>("type","返回类型")
        };
        
    }
    NodeStatus onStart() override
    {
        getInput("type",type);
        getInput("time_",temp);
        timeout(temp);
        timeout.reset();
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {  
        if(type==1)
       {
           if(!timeout.is_timeout())
           {
              return NodeStatus::RUNNING;
           }
           
           else 
           {
                return NodeStatus::SUCCESS;//配合sequence使用
           }
       }
       else
       {
            if(!timeout.is_timeout())
            {
                return NodeStatus::RUNNING;
            }
            
            else 
            {
                return NodeStatus::FAILURE;//配合fallback使用
            }
       }
       
    }

    void onHalted() override
    {
       timeout.stop();
       cout<<"计时结束"<<endl;
    };
};

