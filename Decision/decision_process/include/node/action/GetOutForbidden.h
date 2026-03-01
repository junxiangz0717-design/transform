#pragma once
// 未加姿态

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <vector>
#include "semantic_map/include/semantic_map.h"

using namespace std;
using namespace BT;

class GetOutForbidden : public SyncActionNode
{
public:
    GetOutForbidden(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config),
    points({
            make_shared<const_Point>(11.0, 0.0),
            make_shared<const_Point>(0.0, 0.0),
            make_shared<const_Point>(0.0, 1.0)
        })
    {}
    ~GetOutForbidden()override=default;
    static PortsList providedPorts()
    {
        return {OutputPort<vector<double>>("point","脱离点")};
    }
    NodeStatus tick() override
    {
        double min_dist = numeric_limits<double>::max();
        vector<double> out_put_point;
        for(auto &point : points)
        {
            const_PointPtr closest_point = nullptr;
            double dist = DD.self_pos->distance(point);
            if(dist < min_dist)
            {
                min_dist = dist;
                out_put_point.push_back(point->get_x());
                out_put_point.push_back(point->get_y());
            }
        }
        setOutput("point",out_put_point);
        return NodeStatus::SUCCESS;
    }

private:
    vector<const_PointPtr>points;
};
