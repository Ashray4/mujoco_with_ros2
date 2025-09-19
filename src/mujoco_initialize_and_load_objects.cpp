// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    mujoco_initialize_and_load_objects.cpp
 *
 * \author  Saksham Kohli <kohli@rptu.de>
 * \date    2025/09/10
 *
 */
//-----------------------------------------------------------------------------


#include "mujoco_with_ros2/mujoco_initialize_and_load_objects.h"
#include <filesystem>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

std::mutex mut_ready;
std::condition_variable cv;
bool ready     = false;
bool processed = false;

namespace mujoco_with_ros2 {


std::atomic_bool MujocoInitLoadObjects::alive_{false};
MujocoInitLoadObjects::MujocoInitLoadObjects(std::vector<QueuePtr>& joint_commands_,
                                             std::vector<QueuePtr>& joint_pos_states_,
                                             std::vector<QueuePtr>& joint_vel_states_,
                                             std::vector<QueuePtr>& joint_eff_states_,
                                             const std::vector<int>& joint_ids_,
                                             bool single_thread = false)
  : joint_commands_(joint_commands_)
  , joint_pos_states_(joint_pos_states_)
  , joint_vel_states_(joint_vel_states_)
  , joint_eff_states_(joint_eff_states_)
  , ur5e_joint_ids(joint_ids_)
  , single_thread(single_thread)
{
  bool expected = false;
  loaded_object_instance_ = this;
  if (!alive_.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
    throw std::runtime_error("Only one Mujoco Simulation may exist at a time");
}
MujocoInitLoadObjects::~MujocoInitLoadObjects()
{
  alive_.store(false, std::memory_order_release);
}

// mouse button callback
void MujocoInitLoadObjects::addCallbacks()
{
  // g_self = this;
  // glfwSetMouseButtonCallback(window_, mouseButtonCB);
  // glfwSetCursorPosCallback(window_, mouseMoveCB);
  // glfwSetScrollCallback(window_, scrollCB);

  mjcb_control = controlCB;
}
void MujocoInitLoadObjects::mouseButtonCBImpl(GLFWwindow* window,
                                              [[maybe_unused]] int button,
                                              [[maybe_unused]] int act,
                                              [[maybe_unused]] int mods)
{
  // update button state
  button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


void MujocoInitLoadObjects::mouseMoveCBImpl(GLFWwindow* window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right)
  {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx     = xpos;
  lasty     = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right)
  {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  }
  else if (button_left)
  {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  else
  {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}


void MujocoInitLoadObjects::scrollCBImpl([[maybe_unused]] GLFWwindow* window,
                                         [[maybe_unused]] double xoffset,
                                         double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void MujocoInitLoadObjects::controlCBImpl(const mjModel* m, mjData* d)
{
  // Check if controls are equal

  // std::cout << std::endl<<m->nu<<std::endl<<std::flush;
  // std::cout << std::endl<<d->ctrl<<std::endl<<std::flush;

  for (int i = 0; i < 6; ++i)
  {
    d->ctrl[i]                  = joint_positions_input[i];
    int joint_id                = mj_name2id(m, mjOBJ_JOINT, ur5e_joint_names[i].c_str());
    joint_positions_state[i]    = d->qpos[m->jnt_qposadr[joint_id]];
    joint_velocity_state[i]     = d->qvel[m->jnt_dofadr[joint_id]];
    joint_acceleration_state[i] = d->act[m->jnt_dofadr[joint_id]];
    time_state[i]               = d->time;

    // std::cout << std::endl << "joint_positions_state[i]" << joint_positions_state[i] <<
    // std::flush; std::cout << std::endl << "joint_velocity_state[i]" << joint_velocity_state[i] <<
    // std::flush; std::cout << std::endl
    //           << "joint_acceleration_state[i]" << joint_acceleration_state[i] << std::flush;

    // joint_velocity_state[i] = d->qpvel
    // joint_acceleration_state = d->q
  }
}

mjModel* MujocoInitLoadObjects::initialize_simulation()
{
  // (Test) Load XML manually for now and test the model
  try
  {
    char err_str[1000];
    int err_str_sz = 1000;
    spec           = mj_parseXML(
      "/home/saksham/checkout/thesis_ws/colcon_ws/src/mujoco_with_ros2/models/ur5e/urdf/scene.xml",
      NULL,
      err_str,
      err_str_sz);
    if (!spec)
    {
      std::cout << std::flush << "Problem with model" << std::endl;
      std::cout << std::flush << err_str << std::endl;
      return nullptr;
    }

    // To:Do Possible Object creation and spec editing here

    // spec->option.disableactuator = 1;
    // spec->option.disableactuator = 2;

    m               = mj_compile(spec, NULL);
    m->opt.timestep = 0.002;
    d               = mj_makeData(m);

    // Initialize the buffers
    joint_positions_state.resize(m->nq, 0.0);
    joint_velocity_state.resize(m->nq, 0.0);
    joint_acceleration_state.resize(m->nq, 0.0);
    time_state.resize(m->nq, 0.0);
    joint_positions_input.resize(m->nq, 0.1);
    joint_velocity_input.resize(m->nq, 0.0);
    joint_acceleration_input.resize(m->nq, 0.0);

    std::cout << std::flush << "Simulation Initialized" << std::endl;
    return m;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return nullptr;
  }
}

void MujocoInitLoadObjects::starting_simulation()
{ // check if valid model available

  std::unique_lock<std::mutex> lk(mut_ready);
  if (!single_thread)
  {
    cv.wait(lk, [] { return ready; });
  }

  std::cout << std::flush << "Simulation thread is starting the simulation\n";
  if (m)
  {
    /* code */


    if (!glfwInit())
    {
      mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    window_ = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);


    // install GLFW mouse and keyboard callbacks
    // glfwSetKeyCallback(window, keyboardCB);
    // glfwSetWindowUserPointer(window_, this);
    // g_self = this;
    glfwSetMouseButtonCallback(window_, mouseButtonCB);
    glfwSetCursorPosCallback(window_, mouseMoveCB);
    glfwSetScrollCallback(window_, scrollCB);
    std::cout << std::flush << "No Problems untill callbacks\n";
    
    //addCallbacks();

    if (!d)
    {
      std::cout << std::flush << "Problem here 4(null_ptr!!!!)\n";
    }

    else
    {
      std::cout << std::flush << "not null\n";
    }

    mjcb_control = MujocoInitLoadObjects::controlCB;

    // ... install GLFW keyboard and mouse callbacks

    // run main loop, target real-time simulation and 60 fps rendering

    bool simulation_start = false;
    while (!glfwWindowShouldClose(window_))
    {
      // advance interactive simulation for 1/60 sec
      //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
      //  this loop will finish on time for the next frame to be rendered at 60 fps.
      //  Otherwise add a cpu timer and exit this loop when it is time to render.
      try
      {
        std::cout << std::flush << "No Problems untill loop\n";
        if (!d)
        {
          std::cout << std::flush << "Problem here 4(null_ptr!!!!)\n";
        }

        mjtNum simstart = d->time;
        std::cout << std::flush << "No Problems untill loop 5.1\n";

        while (d->time - simstart < 1.0 / 60.0)
        { 
          std::cout << std::flush << "No Problems untill loop 5.2\n";
          mj_step(m, d);
          std::cout << std::flush << "No Problems untill loop 5.3\n";
        }
        std::cout << std::flush << "No Problems untill loop 5\n";
        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

        std::cout << std::flush << "No Problems untill loop 6\n";
        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        std::cout << std::flush << "No Problems untill loop 7\n";
        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window_);

        std::cout << std::flush << "No Problems untill loop 8\n";

        // // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

        std::cout << std::flush << "No Problems untill loop 9\n";
        if (!simulation_start && !single_thread)
        {
          std::cout << std::flush << "Unlocking and notifying" << std::endl;
          simulation_start = true;
          processed        = true;
          lk.unlock();
          cv.notify_one();
        }
      }
      catch (const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
    }

    if (!single_thread)
    {
      DeletingData();
    }
  }
}

void MujocoInitLoadObjects::controlCB(const mjModel* m, mjData* d)
{
    if (loaded_object_instance_) {
        std::cout << std::flush << "g_self is valid, calling mouseButtonCBImpl..." << std::endl;
        loaded_object_instance_->controlCBImpl(m, d);
    }
}
//try with the obj reference thing
void MujocoInitLoadObjects::mouseButtonCB(GLFWwindow* window, int button, int act, int mods)
{
    if (loaded_object_instance_) {
        loaded_object_instance_->mouseButtonCBImpl(window, button,act,mods);
    }
}

void MujocoInitLoadObjects::mouseMoveCB(GLFWwindow* window, double xpos, double ypos)
{
    if (loaded_object_instance_) {
        std::cout<<std::flush<<std::endl<<loaded_object_instance_->is_deleted;
    }
}
void MujocoInitLoadObjects::scrollCB(GLFWwindow* window, double xoffset, double yoffset)
{
    if (loaded_object_instance_) {
        loaded_object_instance_->scrollCBImpl(window,xoffset,yoffset);
    }
}
void MujocoInitLoadObjects::DeletingData()
{
  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwTerminate();
  is_deleted = true;
}
} // namespace mujoco_with_ros2

// Should be a global main so that the linker finds it, inside the namespace it represents that
// namespace
// int main()
// {
//   // call the static function here
//   mujoco_with_ros2::MujocoInitLoadObjects::init();

//   // start Simulation and visualization
//   mujoco_with_ros2::MujocoInitLoadObjects::start_simulation();

//   // delete data
//   mujoco_with_ros2::MujocoInitLoadObjects::DeleteData();

//   return 0;
// }
