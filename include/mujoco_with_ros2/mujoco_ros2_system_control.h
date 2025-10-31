
#pragma once
#ifndef MUJOCO_ROS2_SYSTEM_H
#  define MUJOCO_ROS2_SYSTEM_H

#  include <bitset>
#  include <chrono>
#  include <cmath>
#  include <cstddef>
#  include <limits>
#  include <memory>
#  include <mutex>
#  include <string>
#  include <thread>
#  include <vector>

#  include "rclcpp/clock.hpp"
#  include "rclcpp/duration.hpp"
#  include "rclcpp/macros.hpp"
#  include "rclcpp/rclcpp.hpp"
#  include "rclcpp/time.hpp"
#  include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#  include "rclcpp_lifecycle/state.hpp"
#  include <realtime_tools/lock_free_queue.hpp>

#  include "hardware_interface/handle.hpp"
#  include "hardware_interface/hardware_info.hpp"
#  include "hardware_interface/system_interface.hpp"
#  include "hardware_interface/types/hardware_interface_return_values.hpp"
#  include "hardware_interface/types/hardware_interface_type_values.hpp"

#  include "mujoco_with_ros2/manage_mujoco.h"
#  include "mujoco_with_ros2/mujoco_initialize_and_load_objects.h"
#  include "mujoco_with_ros2/visibility_control.h"

#  include "GLFW/glfw3.h"
#  include "mujoco/mujoco.h"

using namespace std;

namespace mujoco_with_ros2 {

class MujocowithRos2SystemHardware : public hardware_interface::SystemInterface
{
  virtual ~MujocowithRos2SystemHardware();

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MujocowithRos2SystemHardware)

  MUJOCO_ROS2_SYSTEM_PUBLIC hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo& info) override;

  MUJOCO_ROS2_SYSTEM_PUBLIC std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  MUJOCO_ROS2_SYSTEM_PUBLIC std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  MUJOCO_ROS2_SYSTEM_PUBLIC hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  MUJOCO_ROS2_SYSTEM_PUBLIC hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  MUJOCO_ROS2_SYSTEM_PUBLIC hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  MUJOCO_ROS2_SYSTEM_PUBLIC hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  MUJOCO_ROS2_SYSTEM_PUBLIC hardware_interface::return_type
  read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  MUJOCO_ROS2_SYSTEM_PUBLIC hardware_interface::return_type
  write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  std::vector<double> joint_pos_vector_;
  std::vector<double> joint_vel_vector_;
  std::vector<double> joint_eff_vector_;
  std::vector<double> sensor_vector_;
  std::vector<double> joint_command_pos_vector_;
  std::vector<double> joint_command_vel_vector_;
  std::vector<double> joint_command_eff_vector_;

public:
  std::unique_ptr<ManageMujoco> mujoco_manager;
  CommandTypes command_types_;
  std::vector<std::string> ur5e_joint_names;
  double read_data_;
  double write_data_;
};

} // namespace mujoco_with_ros2

#endif
