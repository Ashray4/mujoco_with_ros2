#pragma once
#ifndef QUEUE_BUFFERS_H
#  define QUEUE_BUFFERS_H

#include <utility>
#include <vector>

#include <bits/stdc++.h>
#include <realtime_tools/lock_free_queue.hpp>

namespace mujoco_with_ros2 {
enum class CommandTypes
{
  POSITION,
  VELOCITY,
  EFFORT,

};
struct QueueBuffers
{
  using QueueType = realtime_tools::LockFreeQueueBase<double, boost::lockfree::spsc_queue<double> >;
  using QueuePtr  = std::unique_ptr<QueueType>;

  int queue_size_;
  size_t num_joints;
  std::vector<QueuePtr> position_values_;
  std::vector<QueuePtr> velocity_values_;
  std::vector<QueuePtr> effort_values_;

  std::string command_name;
  std::vector<CommandTypes> command_types;

  QueueBuffers(int queue_size, size_t n_joints, std::vector<CommandTypes>& command_types)
  {
    queue_size_   = queue_size;
    num_joints    = n_joints;
    command_types = command_types;
    initialize_queue();
  }
  void initialize_queue()
  {
    auto command_size = command_types.size();

    for (int i = 0; i < command_size; i++)
    {
      switch (command_types[i])
      {
        case CommandTypes::POSITION:
          for (int j = 0; j < num_joints; j++)
          {
            position_values_.emplace_back(std::make_unique<QueueType>(queue_size_));
          }
          break;
        case CommandTypes::VELOCITY:
          for (int j = 0; j < num_joints; j++)
          {
             velocity_values_.emplace_back(std::make_unique<QueueType>(queue_size_));
          }
          break;
        case CommandTypes::EFFORT:
          for (int j = 0; j < num_joints; j++)
          {
             effort_values_.emplace_back(std::make_unique<QueueType>(queue_size_));
          }
          break;
        default:
          break;
      }
    }
  }

  std::string& get_name(std::pair<CommandTypes, std::vector<QueuePtr> >& command_pair)
  {
    CommandTypes command_t = command_pair.first;
    switch (command_t)
    {
      case CommandTypes::POSITION:
        command_name = "Position";
        break;
      case CommandTypes::VELOCITY:
        command_name = "Velocity";
        break;
      case CommandTypes::EFFORT:
        command_name = "  Effort";
        break;
      default:
        break;
    }
    return command_name;
  }

  void push_value(CommandTypes type, int index, double value)
  {
    switch (type)
    {
      case CommandTypes::POSITION:
        if (!position_values_.empty())
        {
          static_cast<void>(position_values_[index]->push(value));
        }
        break;
      case CommandTypes::VELOCITY:
        if (!velocity_values_.empty())
        {
          static_cast<void>(velocity_values_[index]->push(value));
        }
        break;
      case CommandTypes::EFFORT:
        if (!effort_values_.empty())
        {
          static_cast<void>(effort_values_[index]->push(value));
        }
        break;
      default:
        break;
    }
  }

  double pop_value(CommandTypes type, int index)
  {
    double data;
    switch (type)
    {
      case CommandTypes::POSITION:
        if (!position_values_.empty())
        {
          static_cast<void>(position_values_[index]->pop(data));
        }
        break;
      case CommandTypes::VELOCITY:
        if (!velocity_values_.empty())
        {
          static_cast<void>(velocity_values_[index]->pop(data));
        }
        break;
      case CommandTypes::EFFORT:
        if (!effort_values_.empty())
        {
          static_cast<void>(effort_values_[index]->pop(data));
        }
        break;
      default:
        break;
    }
    return data;
  }
};
struct CommandBuffer : QueueBuffers
{
  std::vector<CommandTypes> command_type_default{CommandTypes::POSITION};
  // Default constructor for a 6 joint robot
  CommandBuffer()
    : QueueBuffers(1024, 6, command_type_default)
  {
  }
  CommandBuffer(int q_size_, size_t n_joints_, std::vector<CommandTypes>& command_type)
    : QueueBuffers(q_size_, n_joints_, command_type)
  {
  }
};

struct StateBuffer : QueueBuffers
{
  std::vector<CommandTypes> command_type_default{
    CommandTypes::POSITION, CommandTypes::VELOCITY, CommandTypes::EFFORT};
  // Default constructor for a 6 joint robot
  StateBuffer()
    : QueueBuffers(1024, 6, command_type_default)
  {
  }
  StateBuffer(int q_size_, size_t n_joints_, std::vector<CommandTypes>& command_type)
    : QueueBuffers(q_size_, n_joints_, command_type)
  {
  }
};
} // namespace mujoco_with_ros2
#endif
