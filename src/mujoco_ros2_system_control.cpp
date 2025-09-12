#include <mujoco_with_ros2/mujoco_ros2_system_control.h>

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
  current_robot_model = loaded_object_simulation->init();

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)

  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Zero Initialize all joint positions for now (T0:Do read initial positions from the model)
  joint_pos_vector_.resize(info.joints.size(), 0.0);
  joint_vel_vector_.resize(info.joints.size(), 0.0);
  joint_eff_vector_.resize(info.joints.size(), 0.0);

  // REPLACE THE JOINT SIZE WITH ACCURATE JOINTS FROM MUJOCO
  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
                   "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[1].name == hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
                   "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[2].name == hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MujocowithRos2SystemHardware"),
                   "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(),
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
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_EFFORT)
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


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MujocowithRos2SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,&joint_pos_vector_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_vel_vector_[i]));
    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_eff_vector_[i]));
  }


  return command_interfaces;
}
hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Configuring ...please wait...");

  // Call_parameters inverter_c(inverter);

  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Cleaning up ...please wait...");

  con_inv->writeCommandFrequency(0);
  con_inv->stop();
  sleep(2.5);
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Motor speed successfully reset");
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Activating ...please wait...");

  const ModbusConfig& config = {cfg_.port_name.c_str(), cfg_.baud_rate, 'N', 8, 1};

  std::shared_ptr<ModbusProtocol> connection =
    std::make_shared<ModbusProtocol>(config); // shared pointer to the current modbus instance

  con_mod = connection; // connection stays for all time

  // shared pointer for connection with modbus for inverter

  std::shared_ptr<J1000_modbus_parameters> con_temp =
    std::make_shared<J1000_modbus_parameters>(con_mod); // temporory shared pointer with connection

  con_inv = std::make_shared<J1000_modbus_parameters>(con_mod); // inverter with connection
  // con_inv->stop();
  con_inv->setParameter(static_cast<int>(AddressId::MOTOR_CONTROL), 8); // Motor fault reset
  con_inv->writeCommandFrequency(0);
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Successfully activated!");
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"),
              "The velocity range is between 628.31 & 2260.8 rad/s");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Deactivating ...please wait...");

  con_inv->writeCommandFrequency(0);
  con_inv->stop();
  sleep(2.5);
  con_inv.reset();
  con_mod.reset();

  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type MujocowithRos2SystemHardware::read(const rclcpp::Time& /*time*/,
                                                                   const rclcpp::Duration& period)
{
  auto t_fq = con_inv->currentFrequency();
  read_f    = (t_fq / 100) * 6.28;

  read_c = con_inv->currentCurrent();


  std::string binary_t =
    std::bitset<16>(
      static_cast<int>(con_inv->getParameter(static_cast<int>(AddressId::SPEED_AGREE))))
      .to_string();

  read_s = binary_t[13] - '0';
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MujocowithRos2SystemHardware::write(const rclcpp::Time& /*time*/,
                                    const rclcpp::Duration& /*period*/)
{
  double givenFrequency{cfg_.write_velocity};

  count = 0;
  if ((givenFrequency > 628.21 && givenFrequency < 2260.8))
  {
    givenFrequency = (givenFrequency / 6.28) * 100;

    con_inv->writeCommandFrequency(static_cast<int>(givenFrequency));

    con_inv->start();

    sleep(1.5);
  }
  else if (givenFrequency == 0)
  {
    con_inv->writeCommandFrequency(0);
    con_inv->stop();
    sleep(1.5);
  }

  else
  {
    if (count == 0)
    {
      RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"),
                  "The velocity is lower and higher then given limits. Given Velocity: %f",
                  givenFrequency);
      count++;
      sleep(1.5);
    }
  }
  return hardware_interface::return_type::OK;
}

} // namespace mujoco_with_ros2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_with_ros::MujocowithRos2SystemHardware,
                       hardware_interface::SystemInterface)
