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
  joint_pos_vector_.resize(info.joints.size(), 0.0);
  joint_vel_vector_.resize(info.joints.size(), 0.0);
  joint_eff_vector_.resize(info.joints.size(), 0.0);

  joint_command_pos_vector_.resize(info.joints.size(), 0.0);
  joint_command_vel_vector_.resize(info.joints.size(), 0.0);
  joint_command_eff_vector_.resize(info.joints.size(), 0.0);


  // initialize simulation
  ur5e_joint_names.resize(info.joints.size());
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    ur5e_joint_names[i] = info_.joints[i].name;
  }

  // mujoco_manager     = std::make_unique<ManageMujoco>(6,ur5e_joint_names);
  // mujoco_manager->initialize_queues();
  // current_robot_model = mujoco_manager->mujoco_model_;
  // if (!mujoco_manager->joint_commands_[0])
  // {
  //     std::cout<<std::endl<<std::flush<<"Hi i am a null pointer";
  // }
  
  // for (const hardware_interface::ComponentInfo& joint : info_.joints)
  // {

  //   // if (joint.command_interfaces.size() != 3)
  //   // {
  //   //   RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
  //   //                "Joint '%s' has %zu command interfaces found. 1 expected.",
  //   //                joint.name.c_str(),
  //   //                joint.command_interfaces.size());
  //   //   return hardware_interface::CallbackReturn::ERROR;
  //   // }

  //   // if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  //   // {
  //   //   RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
  //   //                "Joint '%s' have %s command interfaces found. '%s' expected.",
  //   //                joint.name.c_str(),
  //   //                joint.command_interfaces[0].name.c_str(),
  //   //                hardware_interface::HW_IF_POSITION);
  //   //   return hardware_interface::CallbackReturn::ERROR;
  //   // }
  //   // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
  //   // {
  //   //   RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
  //   //                "Joint '%s' have %s command interfaces found. '%s' expected.",
  //   //                joint.name.c_str(),
  //   //                joint.command_interfaces[0].name.c_str(),
  //   //                hardware_interface::HW_IF_VELOCITY);
  //   //   return hardware_interface::CallbackReturn::ERROR;
  //   // }
  //   // if (joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
  //   // {
  //   //   RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
  //   //                "Joint '%s' have %s command interfaces found. '%s' expected.",
  //   //                joint.name.c_str(),
  //   //                joint.command_interfaces[0].name.c_str(),
  //   //                hardware_interface::HW_IF_EFFORT);
  //   //   return hardware_interface::CallbackReturn::ERROR;
  //   // }
  //   // if (joint.state_interfaces.size() != 3)
  //   // {
  //   //   RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
  //   //                "Joint '%s' has %zu state interface. 3 expected.",
  //   //                joint.name.c_str(),
  //   //                joint.state_interfaces.size());
  //   //   return hardware_interface::CallbackReturn::ERROR;
  //   // }

  //   // if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  //   // {
  //   //   RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
  //   //                "Joint '%s' have '%s' as a state interface. '%s' expected.",
  //   //                joint.name.c_str(),
  //   //                joint.state_interfaces[0].name.c_str(),
  //   //                hardware_interface::HW_IF_POSITION);
  //   //   return hardware_interface::CallbackReturn::ERROR;
  //   // }
  //   // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
  //   // {
  //   //   RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
  //   //                "Joint '%s' have '%s' as second state interface. '%s' expected.",
  //   //                joint.name.c_str(),
  //   //                joint.state_interfaces[1].name.c_str(),
  //   //                hardware_interface::HW_IF_VELOCITY);
  //   //   return hardware_interface::CallbackReturn::ERROR;
  //   // }
  //   // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_EFFORT)
  //   // {
  //   //   RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
  //   //                "Joint '%s' have '%s' as second state interface. '%s' expected.",
  //   //                joint.name.c_str(),
  //   //                joint.state_interfaces[1].name.c_str(),
  //   //                hardware_interface::HW_IF_EFFORT);
  //   //   return hardware_interface::CallbackReturn::ERROR;
  //   // }
  // }

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


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MujocowithRos2SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_command_pos_vector_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_command_vel_vector_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_command_eff_vector_[i]));
  }


  return command_interfaces;
}
hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Configuring ...please wait...");

  // Initialize simulation with initial joint state

  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Cleaning up ...please wait...");

  // auto& loaded_object_simulation = mujoco_with_ros2::MujocoInitLoadObjects::getInstance();
  // if (!loaded_object_simulation.is_deleted)
  // {
  //   mujoco_with_ros2::MujocoInitLoadObjects::DeleteData();
  // }

  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Activating ...please wait...");
  // mujoco_manager->launch_simulation();
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
  // auto& loaded_object_simulation = mujoco_with_ros2::MujocoInitLoadObjects::getInstance();
  // if (!loaded_object_simulation.is_deleted)
  // {
  //   for (size_t i = 0; i < current_robot_model->nq; i++)
  //   {
  //     joint_pos_vector_[i] = loaded_object_simulation.joint_positions_state[i];
  //     joint_vel_vector_[i] = loaded_object_simulation.joint_velocity_state[i];
  //     joint_eff_vector_[i] = loaded_object_simulation.joint_acceleration_state[i];
  //   }
  //   // std::cout<<loaded_object_simulation.joint_positions_state.size()<<std::flush<<std::endl;
  // }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MujocowithRos2SystemHardware::write(const rclcpp::Time& /*time*/,
                                    const rclcpp::Duration& /*period*/)
{
  // add a check to stop sending values in case simulation stops
  // auto& loaded_object_simulation = mujoco_with_ros2::MujocoInitLoadObjects::getInstance();
  // if (!loaded_object_simulation.is_deleted)
  // {
  //   for (size_t i = 0; i < current_robot_model->nq; i++)
  //   {
  //     loaded_object_simulation.joint_positions_input[i] = joint_command_pos_vector_[i];
  //   }
  // }

  return hardware_interface::return_type::OK;
}

} // namespace mujoco_with_ros2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_with_ros2::MujocowithRos2SystemHardware,
                       hardware_interface::SystemInterface)
