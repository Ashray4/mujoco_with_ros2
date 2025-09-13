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

  //initialize simulation
  auto& loaded_object_simulation = mujoco_with_ros2::MujocoInitLoadObjects::getInstance();
  current_robot_model = loaded_object_simulation.init();
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    std::cout<<info_.joints[i].name.c_str()<<std::endl;
  }

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
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,&joint_command_pos_vector_[i]));

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

  //Initialize simulation with initial joint state
  
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
  thread_ptr->join();
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocowithRos2SystemHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocowithRos2SystemHardware"), "Activating ...please wait...");
  //start the process in another thread
  thread_ptr = std::unique_ptr<std::thread>(new std::thread(mujoco_with_ros2::MujocoInitLoadObjects::start_simulation,std::ref(joint_position_commands)));
  
  //lock the thread and ask the simulation thread to start
  {
  std::lock_guard<std::mutex> lk(mut_ready);
  ready = true;
  std::cout<<"Initializing simulation on the thread id: "<<thread_ptr->get_id()<<'\n';
  }
  cv.notify_one();
  
    // wait for the Simulation-thread to start the simulation
    {
        std::unique_lock<std::mutex> lk(mut_ready);
        cv.wait(lk, []{ return processed; });
    }
  std::cout << "Simulation has been initialized" << '\n';

  
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
  auto& loaded_object_simulation = mujoco_with_ros2::MujocoInitLoadObjects::getInstance();
  joint_pos_vector_ = loaded_object_simulation.joint_positions_state;
  joint_vel_vector_ = loaded_object_simulation.joint_velocity_state;
  joint_eff_vector_ = loaded_object_simulation.joint_acceleration_state;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MujocowithRos2SystemHardware::write(const rclcpp::Time& /*time*/,
                                    const rclcpp::Duration& /*period*/)
{ 
  //add a check to stop sending values in case simulation stops 
  auto& loaded_object_simulation = mujoco_with_ros2::MujocoInitLoadObjects::getInstance();
  for (size_t i = 0; i < current_robot_model->nq; i++)
  {
   loaded_object_simulation.joint_positions_input[i] = joint_command_pos_vector_[i];
  }
  
  return hardware_interface::return_type::OK;
}

} // namespace mujoco_with_ros2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_with_ros2::MujocowithRos2SystemHardware,
                       hardware_interface::SystemInterface)
