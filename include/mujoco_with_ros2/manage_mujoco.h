#include <string>
#include <vector>

#include <mujoco_with_ros2/mujoco_initialize_and_load_objects.h>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

#include <realtime_tools/lock_free_queue.hpp>

namespace mujoco_with_ros2 {
class ManageMujoco
{
  using Queue = std::unique_ptr<
    realtime_tools::LockFreeQueueBase<double, boost::lockfree::spsc_queue<double> > >;

private:
  // command and state values
  int n_joints_;
  std::vector<std::string> mujoco_joint_names_;
  std::vector<int> mujoco_joint_ids_;

  // Mujoco Simulation Variables
  mjData* mujoco_data_;
  size_t queue_size_;
  // thread parameters
  std::unique_ptr<std::thread> thread_ptr;

public:
  ManageMujoco(int n_joints,
               std::vector<std::string>& mujoco_joint_names,
               size_t queue_size = 1024);
  ~ManageMujoco() {};

  int totalJoints();

  const std::vector<std::string>& getJointNames();

  void initialize_queues();
  void launch_simulation(bool single_thread = false);

  bool check_for_instance();
  void delete_simulation();

  MujocoInitLoadObjects& load_mujoco_object_simulation_;
  mjModel* mujoco_model_;
  std::vector<Queue> joint_commands_;
  std::vector<Queue> joint_pos_states_;
  std::vector<Queue> joint_vel_states_;
  std::vector<Queue> joint_eff_states_;
};
} // namespace mujoco_with_ros2
