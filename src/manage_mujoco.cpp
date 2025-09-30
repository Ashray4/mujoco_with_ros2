#include <mujoco_with_ros2/manage_mujoco.h>

namespace mujoco_with_ros2 {

ManageMujoco::ManageMujoco(int n_joints,
                           std::vector<std::string>& mujoco_joint_names,
                           size_t queue_size)
  : n_joints_{n_joints}
  , mujoco_joint_names_{mujoco_joint_names}
  , queue_size_{queue_size}
  , command_buffer_(std::make_shared<CommandBuffer>())
  , state_buffer_(std::make_shared<StateBuffer>())
  , load_mujoco_object_simulation_{MujocoInitLoadObjects::getInstance()}
  , mujoco_model{load_mujoco_object_simulation_.init()}
{
  mujoco_joint_ids_.reserve(n_joints_);
  for (int i = 0; i < n_joints_; i++)
  {
    mujoco_joint_ids_.push_back(
      mj_name2id(mujoco_model, mjOBJ_JOINT, mujoco_joint_names_[i].c_str()));
  }
}

void ManageMujoco::initialize_mujoco_simulation()
{ 
  mujoco_with_ros2::MujocoInitLoadObjects::init();
}
int ManageMujoco::totalJoints()
{
  return n_joints_;
}

const std::vector<std::string>& ManageMujoco::getJointNames()
{
  return mujoco_joint_names_;
}

const std::vector<int>& ManageMujoco::getJointIds()
{
  return mujoco_joint_ids_;
}

// void ManageMujoco::initialize_queues()
// {
//   // initialize the queues vectors
//   joint_commands_.reserve(n_joints_);
//   joint_pos_states_.reserve(n_joints_);
//   joint_vel_states_.reserve(n_joints_);
//   joint_eff_states_.reserve(n_joints_);

//   for (int i = 0; i < n_joints_; i++)
//   {
//     joint_commands_.emplace_back(std::make_unique<QueueType>(queue_size_));
//     joint_pos_states_.emplace_back(std::make_unique<QueueType>(queue_size_));
//     joint_vel_states_.emplace_back(std::make_unique<QueueType>(queue_size_));
//     joint_eff_states_.emplace_back(std::make_unique<QueueType>(queue_size_));
//   }
// }
// void ManageMujoco::launch_simulation(bool single_thread)
// {
//   // start the process in another thread
//   thread_ptr = std::unique_ptr<std::thread>(new std::thread([=] {
//     load_mujoco_object_simulation_.start_simulation(joint_commands_,
//                                                     joint_pos_states_,
//                                                     joint_vel_states_,
//                                                     joint_eff_states_,
//                                                     mujoco_joint_ids_,
//                                                     single_thread);
//   }));

//   // lock the thread and ask the simulation thread to start
//   {
//     std::lock_guard<std::mutex> lk(mut_ready);
//     ready = true;
//     std::cout << "Initializing simulation on the thread id: " << thread_ptr->get_id() << '\n';
//   }

//   cv.notify_one();

//   // wait for the Simulation-thread to start the simulation
//   {
//     std::unique_lock<std::mutex> lk(mut_ready);
//     cv.wait(lk, [] { return processed; });
//   }
//   std::cout << "Simulation has been initialized" << '\n';

//   ready     = false;
//   processed = false;
// }

void ManageMujoco::delete_simulation() {}
} // namespace mujoco_with_ros2
