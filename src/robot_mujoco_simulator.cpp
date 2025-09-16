#include "mujoco_with_ros2/mujoco_initialize_and_load_objects.h"
#include <chrono>
#include <thread>

int main()
{
  // call the static function here
  mujoco_with_ros2::MujocoInitLoadObjects::init();
  // start Simulation and visualization
  mujoco_with_ros2::MujocoInitLoadObjects::start_simulation(true);

  // delete data
  mujoco_with_ros2::MujocoInitLoadObjects::DeleteData();

  return 0;
}
