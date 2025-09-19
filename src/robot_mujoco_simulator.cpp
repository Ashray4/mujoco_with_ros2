#include "mujoco_with_ros2/manage_mujoco.h"
#include <chrono>
#include <thread>
#include <vector>

int main()
{
  // call the static function here
  int n_joints          = 6;
  std::vector<std::string> ur5e_joint_names = {"shoulder_pan_joint",
                           "shoulder_lift_joint",
                           "elbow_joint",
                           "wrist_1_joint",
                           "wrist_2_joint",
                           "wrist_3_joint"};
  mujoco_with_ros2::ManageMujoco mujoco_manager(n_joints,ur5e_joint_names,true);
  
  //initialize the instance
  mujoco_manager.load_mujoco_object_simulation_->getInstance(mujoco_manager.load_mujoco_object_simulation_.get());
  //initializeing simulation
  mujoco_manager.load_mujoco_object_simulation_->initialize_simulation();

  // start Simulation and visualization
  mujoco_manager.load_mujoco_object_simulation_->starting_simulation();

  // // delete data
  mujoco_manager.load_mujoco_object_simulation_->DeletingData();

  return 0;
}
