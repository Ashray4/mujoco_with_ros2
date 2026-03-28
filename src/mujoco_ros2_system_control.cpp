#include <mujoco_with_ros2/mujoco_ros2_system_control.h>
// add another class to manage the communication via named interfaces like command interfaces for
// mujoco
// states are not being published
// make the update rate same
// add publishers and ros2 nodes and services for teleoperation and objects
// try interactive markers in mujoco(teleoperation)

namespace mujoco_with_ros2 {

MujocowithRos2SystemHardware::~MujocowithRos2SystemHardware()
{
  // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
  // We therefore need to make sure to actually deactivate the communication
  on_cleanup(rclcpp_lifecycle::State());
}
hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)

  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Zero Initialize all joint positions for now (T0:Do read initial positions from the model)
  joint_pos_vector_.resize(info_.joints.size(), 0.0);
  joint_vel_vector_.resize(info_.joints.size(), 0.0);
  joint_eff_vector_.resize(info_.joints.size(), 0.0);
  sensor_vector_.resize(6, 0.0); // hardcoded
  body_vector_.resize(6, 0.0);   // hardcoded

  joint_command_pos_vector_.resize(info_.joints.size(), 0.0);
  joint_command_vel_vector_.resize(info_.joints.size(), 0.0);
  joint_command_eff_vector_.resize(info_.joints.size(), 0.0);


  // initialize simulation
  // hardcoded, add the real joints later
  // ur5e_joint_names.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    std::cout << std::endl << info_.joints[i].name << std::endl;
    std::cout << std::endl << joint_command_pos_vector_.size() << std::endl;
  }
  ur5e_joint_names = {"shoulder_pan_joint",
                      "shoulder_lift_joint",
                      "elbow_joint",
                      "wrist_1_joint",
                      "wrist_2_joint",
                      "wrist_3_joint"};
  bool test        = true;

  std::cout << std::flush << info_.joints[0].command_interfaces[0].name << std::endl;
  std::cout << std::flush << info_.joints[1].command_interfaces[0].name << std::endl;
  std::cout << std::flush << info_.joints[2].command_interfaces[0].name << std::endl;

  if (info_.joints[0].command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
  {
    command_types_ = CommandTypes::POSITION;
    std::cout << std::endl << std::flush << "I choose Position" << std::endl;
  }
  if (info_.joints[0].command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)
  {
    command_types_ = CommandTypes::VELOCITY;
  }
  if (info_.joints[0].command_interfaces[0].name == hardware_interface::HW_IF_EFFORT)
  {
    command_types_ = CommandTypes::EFFORT;
    std::cout << std::endl << std::flush << "I choose Effort" << std::endl;
  }
  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if ((joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
        (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) &&
        (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
                   "Joint '%s' have %s command interfaces found. '%s','%s' or '%s' expected.",
                   joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY,
                   hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
                   "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(),
                   joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
                   "Joint '%s' have '%s' as a state interface. '%s' expected.",
                   joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
                   "Joint '%s' have '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
                   "Joint '%s' have '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MujocowithRos2SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_pos_vector_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_vel_vector_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_eff_vector_[i]));
  }

  // expose force.x, force.y, force.z
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("motor_fts", "force.x", &sensor_vector_[0]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("motor_fts", "force.y", &sensor_vector_[1]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("motor_fts", "force.z", &sensor_vector_[2]));

  // expose torque.x, torque.y, torque.z
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("motor_fts", "torque.x", &sensor_vector_[3]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("motor_fts", "torque.y", &sensor_vector_[4]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("motor_fts", "torque.z", &sensor_vector_[5]));

  // hardcoded
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("body1", "pos.x", &body_vector_[0]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("body1", "pos.y", &body_vector_[1]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("body1", "pos.z", &body_vector_[2]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("body2", "pos.x", &body_vector_[3]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("body2", "pos.y", &body_vector_[4]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("body2", "pos.z", &body_vector_[5]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MujocowithRos2SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_command_pos_vector_[i]));
      std::cout << std::endl << info_.joints[i].name << std ::endl;
    }

    if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_command_vel_vector_[i]));
    }
    if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_EFFORT)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_command_eff_vector_[i]));
    }
    std::cout << std::endl << "joint size: " << joint_command_pos_vector_.size() << std::endl;
  }

  return command_interfaces;
}
hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Configuring ...please wait...");

  // Initialize simulation with initial joint state


  mujoco_manager = std::make_unique<ManageMujoco>(6, ur5e_joint_names, command_types_);

  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Cleaning up ...please wait...");

  auto& loaded_object_simulation = mujoco_with_ros2::MujocoInitLoadObjects::getInstance();
  if (!loaded_object_simulation.is_deleted)
  {
    mujoco_with_ros2::MujocoInitLoadObjects::DeleteData();
  }

  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Activating ...please wait...");
  mujoco_manager->launch_simulation();
  joint_command_pos_vector_ = mujoco_manager.get()->joint_init_pos_vector_;
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type MujocowithRos2SystemHardware::read(const rclcpp::Time& /*time*/,
                                                                   const rclcpp::Duration& period)
{
  for (size_t i = 0; i < ur5e_joint_names.size(); i++)
  {
    if (mujoco_manager->state_buffer_->pop_value(
          mujoco_with_ros2::CommandTypes::POSITION, i, read_data_))
    {
      joint_pos_vector_[i] = read_data_;
    }
    if (mujoco_manager->state_buffer_->pop_value(
          mujoco_with_ros2::CommandTypes::VELOCITY, i, read_data_))
    {
      joint_vel_vector_[i] = read_data_;
    }
    if (mujoco_manager->state_buffer_->pop_value(
          mujoco_with_ros2::CommandTypes::EFFORT, i, read_data_))
    {
      joint_eff_vector_[i] = read_data_;
    }
  }

  for (size_t i = 0; i < 6; i++)
  {
    if (mujoco_manager->sensor_buffer_->pop_value(
          mujoco_with_ros2::CommandTypes::SENSOR, i, read_data_))
    {
      sensor_vector_[i] = read_data_;
    }
  }

  // hardcoded
  if (mujoco_manager->eef_state_buffer_->pop_value(
        mujoco_with_ros2::CommandTypes::POSITION, 0, read_data_))
  {
    joint_pos_vector_[joint_pos_vector_.size() - 1] = read_data_;
  }
  if (mujoco_manager->eef_state_buffer_->pop_value(
        mujoco_with_ros2::CommandTypes::VELOCITY, 0, read_data_))
  {
    joint_vel_vector_[joint_pos_vector_.size() - 1] = read_data_;
  }
  if (mujoco_manager->state_buffer_->pop_value(
        mujoco_with_ros2::CommandTypes::EFFORT, 0, read_data_))
  {
    joint_eff_vector_[joint_pos_vector_.size() - 1] = read_data_;
  }
  // hardcoded
  for (size_t i = 0; i < 6; i++)
  {
    if (mujoco_manager->body_state_buffer_->pop_value(
          mujoco_with_ros2::CommandTypes::POSITION, i, read_data_))
    {
      body_vector_[i] = read_data_;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MujocowithRos2SystemHardware::write(const rclcpp::Time& /*time*/,
                                    const rclcpp::Duration& /*period*/)
{
  for (size_t i = 0; i < ur5e_joint_names.size(); i++)
  {
    bool success = false;

    if (command_types_ == mujoco_with_ros2::CommandTypes::POSITION)
    {
      success = mujoco_manager->command_buffer_->push_value(
        command_types_, i, joint_command_pos_vector_[i]);
      // std::cout << std::flush << "Pushing position[" << i << "]: " <<
      // joint_command_pos_vector_[i]
      //           << " success: " << success << std::endl;
    }
    else if (command_types_ == mujoco_with_ros2::CommandTypes::VELOCITY)
    {
      success = mujoco_manager->command_buffer_->push_value(
        command_types_, i, joint_command_vel_vector_[i]);
    }
    else if (command_types_ == mujoco_with_ros2::CommandTypes::EFFORT)
    {
      success = mujoco_manager->command_buffer_->push_value(
        command_types_, i, joint_command_eff_vector_[i]);
      // std::cout << std::flush << "Pushing effort[" << i << "]: "
      //           << joint_command_eff_vector_[i] << " success: " << success << std::endl;
    }
  }

  // hardcoded only supports position interface for now
  auto ss = mujoco_manager->eef_command_buffer_->push_value(
    mujoco_with_ros2::CommandTypes::POSITION, 0, joint_command_pos_vector_[6]);
  // std::cout << std::flush
  //           << "Size["
  //              "]: "
  //           << joint_command_pos_vector_.size() << std::endl;
  // std::cout << std::flush
  //           << "eef["
  //              "]: "
  //           << joint_command_pos_vector_.back() << std::endl;
  return hardware_interface::return_type::OK;
}

} // namespace mujoco_with_ros2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_with_ros2::MujocowithRos2SystemHardware,
                       hardware_interface::SystemInterface)
