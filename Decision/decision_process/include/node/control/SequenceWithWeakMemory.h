//
// Created by zh on 24-3-27.
//
#pragma once

#include "behaviortree_cpp/control_node.h"

namespace BT
{
 /**
 * @brief 和BT.CPP内置的SequenceWithMemory唯一不同的就是会在被中断时将执行序号复位，下次从头开始执行
 *        所以取名为“弱记忆序列”
 */
class SequenceWithWeakMemory : public ControlNode
{
public:
  SequenceWithWeakMemory(const std::string& name);

  ~SequenceWithWeakMemory() override = default;

  void halt() override;

private:
  size_t current_child_idx_;
  bool all_skipped_ = true;

  virtual BT::NodeStatus tick() override;
};
}   // namespace BT

namespace BT
{
SequenceWithWeakMemory::SequenceWithWeakMemory(const std::string& name) :
  ControlNode::ControlNode(name, {}), current_child_idx_(0)
{
  setRegistrationID("SequenceWithWeakMemory");
}

NodeStatus SequenceWithWeakMemory::tick()
{
  const size_t children_count = children_nodes_.size();

  if(status() == NodeStatus::IDLE)
  {
    all_skipped_ = true;
  }
  setStatus(NodeStatus::RUNNING);

  while (current_child_idx_ < children_count)
  {
    TreeNode* current_child_node = children_nodes_[current_child_idx_];

    auto prev_status = current_child_node->status();
    const NodeStatus child_status = current_child_node->executeTick();

    // switch to RUNNING state as soon as you find an active child
    all_skipped_ &= (child_status == NodeStatus::SKIPPED);

    switch (child_status)
    {
      case NodeStatus::RUNNING: {
        return child_status;
      }
      case NodeStatus::FAILURE: {
        // DO NOT reset current_child_idx_ on failure
        for (size_t i = current_child_idx_; i < childrenCount(); i++)
        {
          haltChild(i);
        }

        return child_status;
      }
      case NodeStatus::SUCCESS: {
        current_child_idx_++;
        // Return the execution flow if the child is async,
        // to make this interruptable.
        if (requiresWakeUp() && prev_status == NodeStatus::IDLE &&
            current_child_idx_ < children_count)
        {
          emitWakeUpSignal();
          return NodeStatus::RUNNING;
        }
      }
      break;

      case NodeStatus::SKIPPED: {
        // It was requested to skip this node
        current_child_idx_++;
      }
      break;

      case NodeStatus::IDLE: {
        throw LogicError("[", name(), "]: A children should not return IDLE");
      }
    }   // end switch
  }     // end while loop

  // The entire while loop completed. This means that all the children returned SUCCESS.
  if (current_child_idx_ == children_count)
  {
    resetChildren();
    current_child_idx_ = 0;
  }
  // Skip if ALL the nodes have been skipped
  return all_skipped_ ? NodeStatus::SKIPPED : NodeStatus::SUCCESS;
}

void SequenceWithWeakMemory::halt()
{
  // 和SequenceWithMemory唯一不同的就是会在被中断时将执行序号复位，下次从头开始执行
  current_child_idx_ = 0;
  ControlNode::halt();
}

}   // namespace BT