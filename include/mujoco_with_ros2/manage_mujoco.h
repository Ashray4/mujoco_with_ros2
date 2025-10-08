#include <string>
#include <thread>
#include <vector>
#include <memory>

#include <mujoco_with_ros2/command_and_state_buffer.h>
#include <mujoco_with_ros2/mujoco_initialize_and_load_objects.h>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

namespace mujoco_with_ros2 {
class ManageMujoco
{
private:
  // command and state values
  int n_joints_;
  std::vector<std::string> mujoco_joint_names_;
  std::vector<int> mujoco_joint_ids_;

  // Mujoco Simulation Variables
  size_t queue_size_;


  // thread parameters
  std::unique_ptr<std::thread> thread_ptr;

public:
  ManageMujoco(int n_joints,
               std::vector<std::string>& mujoco_joint_names,CommandTypes control_mode = CommandTypes::POSITION,
               size_t queue_size = 1024);
  ~ManageMujoco();

  int totalJoints();

  void initialize_mujoco_simulation();
  const std::vector<std::string>& getJointNames();
  const std::vector<int>& getJointIds();

  void initialize_queues();
  void launch_simulation(bool single_thread = false);

  bool check_for_instance();
  void delete_simulation();

  // MuJoCo data structures
  mjSpec* mujoco_spec   = NULL; // MuJoCo Spec
  mjModel* mujoco_model = NULL; // MuJoCo model
  mjData* mujoco_data   = NULL; // MuJoCo data
  
  MujocoInitLoadObjects& load_mujoco_object_simulation_;
  
  std::shared_ptr<CommandBuffer> command_buffer_;
  std::shared_ptr<StateBuffer> state_buffer_;
  CommandTypes control_mode_;
};
} // namespace mujoco_with_ros2
