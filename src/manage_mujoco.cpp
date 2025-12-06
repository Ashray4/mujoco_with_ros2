#include <mujoco_with_ros2/manage_mujoco.h>

namespace mujoco_with_ros2 {

ManageMujoco::ManageMujoco(int n_joints,
                           std::vector<std::string>& mujoco_joint_names,
                           CommandTypes control_mode,
                           size_t queue_size,
                           std::string end_effector)
  : n_joints_{n_joints}
  , mujoco_joint_names_{mujoco_joint_names}
  , control_mode_(control_mode)
  , queue_size_{queue_size}
  , load_mujoco_object_simulation_{MujocoInitLoadObjects::getInstance()}
  , command_buffer_(
      std::make_shared<CommandBuffer>(queue_size_, n_joints_, std::vector<CommandTypes>{control_mode}))
  , state_buffer_(std::make_shared<StateBuffer>())
  , sensor_buffer_(
      std::make_shared<StateBuffer>(queue_size_, 6, std::vector<CommandTypes>{CommandTypes::SENSOR}))
  , tool(end_effector)
{
  initialize_mujoco_simulation();
  mujoco_joint_ids_.reserve(n_joints_);

  // Get Joint Ids from joint names
  for (int i = 0; i < n_joints_; i++)
  {
    mujoco_joint_ids_.push_back(
      mj_name2id(mujoco_model, mjOBJ_JOINT, mujoco_joint_names_[i].c_str()));
    // initialize commands to 0 and add the command type logic later
    command_buffer_->push_value(control_mode_, i, 0.0);
  }

  // TO:Do ( later automated way of doing this ) get sensor information ( size 3 )
  mujoco_sensor_ids_.reserve(6);

  for (int i = 0; i < mujoco_model->nsensor; i++)
  { 
    std::cout<<std::flush<<mj_id2name(mujoco_model, mjOBJ_SENSOR, i)<<std::endl;
    if ( std::string(mj_id2name(mujoco_model, mjOBJ_SENSOR, i)) == "motor_force" ||
        std::string(mj_id2name(mujoco_model, mjOBJ_SENSOR, i)) == "motor_torque")
    {
      int dim = mujoco_model->sensor_dim[i];
      for (int j = 0; j < dim; j++)
      { int sum = i + j;
        if (i > 0)
        {
          sum = ( i - 1 ) + j + mujoco_model->sensor_dim[0];
        }       
        mujoco_sensor_ids_.push_back(sum);
      }
    }
  }
  
  for (size_t i = 0; i < mujoco_sensor_ids_.size(); i++)
  {
    std::cout<<std::flush<<mujoco_sensor_ids_[i]<<std::endl;
  }
  
  load_mujoco_object_simulation_.initialize_buffers(
    command_buffer_, state_buffer_, sensor_buffer_, mujoco_joint_ids_, mujoco_sensor_ids_);
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

  mujoco_model = mj_compile(mujoco_spec, NULL);
  if (!mujoco_model)
  {
    std::cout << "Failed to compile model" << std::endl;
  }
  mujoco_data = mj_makeData(mujoco_model);

  int actuator_mode = 0; // Change this to select mode

  if (control_mode_ == CommandTypes::POSITION)
  {
    actuator_mode = 0;
  }
  else if (control_mode_ == CommandTypes::VELOCITY)
  {
    actuator_mode = 1;
  }
  else if (control_mode_ == CommandTypes::EFFORT)
  {
    actuator_mode = 2;
  }
  std::cout << std::flush << "actuator_mode: " << actuator_mode << std::endl;

  // Disable all actuators first
  for (int i = 0; i < mujoco_model->nu; i++)
  {
    mujoco_model->actuator_actlimited[i] = 0;
    // std::cout << std::flush << i << std::endl;
  }

  // Enable only the selected group
  for (int i = 0; i < mujoco_model->nu; i++)
  {
    if (mujoco_model->actuator_group[i] == actuator_mode)
    {
      mujoco_model->actuator_actlimited[i] = 1; // Enable this actuator
      std::cout << std::flush << i << std::endl;
    }
  }

  mujoco_with_ros2::MujocoInitLoadObjects::init(mujoco_model, mujoco_data);
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
