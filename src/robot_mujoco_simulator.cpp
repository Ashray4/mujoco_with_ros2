#include "mujoco_with_ros2/manage_mujoco.h"
#include <chrono>
#include <thread>
#include <vector>
int main()
{
  // call the static function here
  int n_joints                              = 6;
  std::vector<std::string> ur5e_joint_names = {"shoulder_pan_joint",
                                               "shoulder_lift_joint",
                                               "elbow_joint",
                                               "wrist_1_joint",
                                               "wrist_2_joint",
                                               "wrist_3_joint"};
  std::cout << std::flush << "starting " << std::endl;
  mujoco_with_ros2::ManageMujoco mujoco_manager(n_joints, ur5e_joint_names);

  // // start Simulation and visualization
  mujoco_manager.initialize_mujoco_simulation();
  mujoco_manager.load_mujoco_object_simulation_.start_simulation(true);

  // // // delete data
  mujoco_manager.load_mujoco_object_simulation_.DeleteData();

  return 0;
}
