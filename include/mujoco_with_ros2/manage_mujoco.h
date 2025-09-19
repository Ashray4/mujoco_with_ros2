#include <string>
#include <thread>
#include <vector>

#include <mujoco_with_ros2/mujoco_initialize_and_load_objects.h>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

#include <realtime_tools/lock_free_queue.hpp>

namespace mujoco_with_ros2 {
class ManageMujoco
{
  using QueueType = realtime_tools::LockFreeQueueBase<double, boost::lockfree::spsc_queue<double> >;
  using QueuePtr  = std::unique_ptr<QueueType>;

private:
  // command and state values
  int n_joints_;
  std::vector<std::string> mujoco_joint_names_;
  std::vector<int> mujoco_joint_ids_;

  // Mujoco Simulation Variables
  size_t queue_size_;
  bool single_thread_;

  // thread parameters
  std::unique_ptr<std::thread> thread_ptr;

public:
  ManageMujoco(int n_joints,
               std::vector<std::string>& mujoco_joint_names,bool single_thread,
               size_t queue_size = 1024);
  ~ManageMujoco() {};

  int totalJoints();

  mjModel* initialize_mujoco_model();
  const std::vector<std::string>& getJointNames();
  const std::vector<int>& getJointIds();

  void initialize_queues();
  void launch_simulation(bool single_thread = false);

  bool check_for_instance(){};
  void delete_simulation();

  // MuJoCo data structures
  mjSpec* mujoco_spec = NULL; // MuJoCo Spec
  mjModel* mujoco_model   = NULL; // MuJoCo model
  mjData* mujoco_data    = NULL; // MuJoCo data

   std::unique_ptr<MujocoInitLoadObjects> load_mujoco_object_simulation_;

  std::vector<QueuePtr> joint_commands_;
  std::vector<QueuePtr> joint_pos_states_;
  std::vector<QueuePtr> joint_vel_states_;
  std::vector<QueuePtr> joint_eff_states_;
};
} // namespace mujoco_with_ros2
