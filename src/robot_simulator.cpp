#include "mujoco_with_ros2/mujoco_initialize_and_load_objects.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include <chrono>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Start the simulator in parallel.
  // Let the thread's destructor clean-up all resources
  // once users close the simulation window.
  auto simulator = std::thread(mujoco_with_ros2::MujocoInitLoadObjects::simulate);
  simulator.detach();

  while (!mujoco_with_ros2::MujocoInitLoadObjects::ready())
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  rclcpp::spin(mujoco_with_ros2::MujocoInitLoadObjects::getNode());
  rclcpp::shutdown();
  return 0;
}
