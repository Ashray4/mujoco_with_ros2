// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    mujoco_simulator.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/09/27
 *
 */
//-----------------------------------------------------------------------------


#include "mujoco_with_ros2/mujoco_initialize_and_load_objects.h"
#include <filesystem>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

namespace mujoco_with_ros2 {

MujocoInitLoadObjects::MujocoInitLoadObjects() {}

int MujocoInitLoadObjects::init()
{
  std::cout << std::flush << "Simulation Initialized" << std::endl;
  return getInstance().initialize_simulation();
}
int MujocoInitLoadObjects::initialize_simulation()
{
  // (Test) Load XML manually for now and test the model
  try
  {
    char* err_str;
    int err_str_sz = 10;
    mjModel* m = mj_loadXML(
      "/home/saksham/checkout/thesis_ws/colcon_ws/src/mujoco_with_ros2/models/ur5e/urdf/ur5e.xml",
      NULL,
      err_str,
      err_str_sz);
    if (!m)
    {
      std::cout << std::flush << "Problem with model" << std::endl;
      std::cout << std::flush << err_str << std::endl;
      return 0;
    }

    mjData* d = mj_makeData(m);

    std::cout << std::flush << "Error here 3" << std::endl;
    while (d->time < 10)
      mj_step(m, d);

    std::cout << std::flush << "Simulation Done" << std::endl;

    // deallocate existing mjModel
    mj_deleteModel(m);

    // deallocate existing mjData
    mj_deleteData(d);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }


  return 1;
};
} // namespace mujoco_with_ros2

// Should be a global main so that the linker finds it, inside the namespace it represents that
// namespace
int main()
{
  std::cout << std::flush << "Staring Simulation" << std::endl;
  // call the static function here
  auto simulator = mujoco_with_ros2::MujocoInitLoadObjects::init();
  return 0;
}
