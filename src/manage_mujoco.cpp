#include <mujoco_with_ros2/manage_mujoco.h>

namespace mujoco_with_ros2 {

ManageMujoco::ManageMujoco(int n_joints, std::vector<std::string>& mujoco_joint_names)
  : n_joints_{n_joints},
  mujoco_joint_names_{mujoco_joint_names},
  load_mujoco_object_simulation_{MujocoInitLoadObjects::getInstance()},
  mujoco_model_{load_mujoco_object_simulation_.init()}
{

}
} // namespace mujoco_with_ros2
