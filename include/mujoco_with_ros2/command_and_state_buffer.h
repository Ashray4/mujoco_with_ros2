#pragma once
#ifndef QUEUE_BUFFERS_H
#  define QUEUE_BUFFERS_H

#  include <utility>
#  include <vector>

#  include <bits/stdc++.h>
#  include <realtime_tools/lock_free_queue.hpp>

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
  std::vector<double> prev_data;
  size_t num_joints;
  std::vector<QueuePtr> position_values_;
  std::vector<QueuePtr> velocity_values_;
  std::vector<QueuePtr> effort_values_;

  std::string command_name;
  std::vector<CommandTypes> command_types_;

  QueueBuffers(int queue_size, size_t n_joints, std::vector<CommandTypes> command_types)
  {
    queue_size_    = queue_size;
    num_joints     = n_joints;
    command_types_ = command_types;
    initialize_queue();
    prev_data.resize(n_joints);
  }
  void initialize_queue()
  {
    auto command_size = command_types_.size();

    for (int i = 0; i < command_size; i++)
    {
      switch (command_types_[i])
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
            std::cout<<std::flush<<"i choose effort"<<std::endl;
          }
          break;
        default:
          break;
      }
    }
  }
  CommandTypes get_command_type() { return command_types_[0]; }
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
        // std::cout <<std::flush<<std::endl<< "COULDN'T PUSH BECAUSE WRONG COMMAND TYPE";
        break;
    }
  }

  bool pop_value(CommandTypes type, int index, double& data)
  {
    bool success = false;
    switch (type)
    {
      case CommandTypes::POSITION:
        if (!position_values_.empty())
        {
          success = position_values_[index]->pop(data);
        }
        break;
      case CommandTypes::VELOCITY:
        if (!velocity_values_.empty())
        {
          success = velocity_values_[index]->pop(data);
        }
        break;
      case CommandTypes::EFFORT:
        if (!effort_values_.empty())
        {
          success = effort_values_[index]->pop(data);
          //std::cout <<std::flush<< data<<std::endl;
        }
        break;
      default:
        break;
    }
    //std::cout<<std::flush<<"after: "<<success<<std::endl;
    return success;
  }
};
struct CommandBuffer : QueueBuffers
{
  // Default constructor for a 6 joint robot
  CommandBuffer()
    : QueueBuffers(1024, 6, {CommandTypes::POSITION})
  {
  }
  CommandBuffer(int q_size_, size_t n_joints_, std::vector<CommandTypes> command_type)
    : QueueBuffers(q_size_, n_joints_, command_type)
  {
  }
};

struct StateBuffer : QueueBuffers
{
  // Default constructor for a 6 joint robot
  StateBuffer()
    : QueueBuffers(1024, 6, {CommandTypes::POSITION, CommandTypes::VELOCITY, CommandTypes::EFFORT})
  {
  }
  StateBuffer(int q_size_, size_t n_joints_, std::vector<CommandTypes> command_type)
    : QueueBuffers(q_size_, n_joints_, command_type)
  {
  }
};
} // namespace mujoco_with_ros2
#endif
