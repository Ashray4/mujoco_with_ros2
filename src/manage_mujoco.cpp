#include <mujoco_with_ros2/manage_mujoco.h>

namespace mujoco_with_ros2 {

ManageMujoco::ManageMujoco(int n_joints,
                           std::vector<std::string>& mujoco_joint_names,
                           size_t queue_size)
  : n_joints_{n_joints}
  , mujoco_joint_names_{mujoco_joint_names}
  , queue_size_{queue_size}
  , load_mujoco_object_simulation_{MujocoInitLoadObjects::getInstance()}
  , command_buffer_(std::make_shared<CommandBuffer>())
  , state_buffer_(std::make_shared<StateBuffer>())
{
  load_mujoco_object_simulation_.initialize_buffers(command_buffer_, state_buffer_);
  initialize_mujoco_simulation();
  mujoco_joint_ids_.reserve(n_joints_);
  for (int i = 0; i < n_joints_; i++)
  {
    mujoco_joint_ids_.push_back(
      mj_name2id(mujoco_model, mjOBJ_JOINT, mujoco_joint_names_[i].c_str()));

    // initialize commands to 0 and add the command type logic later
    command_buffer_->push_value(CommandTypes::POSITION, i, 0.3);
  }
}
ManageMujoco::~ManageMujoco()
{
  thread_ptr->join();
}
void ManageMujoco::initialize_mujoco_simulation()
{
  char err_str[1000];
  int err_str_sz = 1000;
  mujoco_spec    = mj_parseXML(
    "/home/saksham/checkout/thesis_ws/colcon_ws/src/mujoco_with_ros2/models/ur5e/urdf/scene.xml",
    NULL,
    err_str,
    err_str_sz);
  if (!mujoco_spec)
  {
    std::cout << std::flush << "Problem with model" << std::endl;
    std::cout << std::flush << err_str << std::endl;
  }

  // To:Do Possible Object creation and spec editing here

  mujoco_spec->option.disableactuator = 1;
  mujoco_spec->option.disableactuator = 2;

  mujoco_model = mj_compile(mujoco_spec, NULL);
  mujoco_data  = mj_makeData(mujoco_model);

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

void ManageMujoco::launch_simulation(bool single_thread)
{
  // start the process in another thread
  thread_ptr = std::unique_ptr<std::thread>(
    new std::thread([=] { load_mujoco_object_simulation_.start_simulation(single_thread); }));

  // lock the thread and ask the simulation thread to start
  {
    std::lock_guard<std::mutex> lk(mut_ready);
    ready = true;
    std::cout << "Initializing simulation on the thread id: " << thread_ptr->get_id() << '\n';
  }

  cv.notify_one();

  // wait for the Simulation-thread to start the simulation
  {
    std::unique_lock<std::mutex> lk(mut_ready);
    cv.wait(lk, [] { return processed; });
  }
  std::cout << "Simulation has started" << '\n';

  ready     = false;
  processed = false;
}

void ManageMujoco::delete_simulation() {}
} // namespace mujoco_with_ros2
