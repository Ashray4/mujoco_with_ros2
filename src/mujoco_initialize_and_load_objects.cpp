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

MujocoInitLoadObjects::MujocoInitLoadObjects() {}

void MujocoInitLoadObjects::initialize_buffers(std::shared_ptr<CommandBuffer> c_buff,
                                               std::shared_ptr<StateBuffer> s_buff,
                                               std::shared_ptr<StateBuffer> sens_buff,
                                               std::vector<int> joint_ids_,
                                               std::vector<int> sensor_ids_)
{
  command_buffer_    = c_buff;
  state_buffer_      = s_buff;
  mujoco_joint_ids_  = joint_ids_;
  mujoco_sensor_ids_ = sensor_ids_;
  sensor_buffer_     = sens_buff;
  command_type_      = command_buffer_->get_command_type();
}
void MujocoInitLoadObjects::updateUI()
{
  // Update visualization options from checkboxes
  if (show_contact)
  {
    vopt.flags[mjVIS_CONTACTPOINT] = 1;
    vopt.flags[mjVIS_CONTACTFORCE] = 1;
  }
  else
  {
    vopt.flags[mjVIS_CONTACTPOINT] = 0;
    vopt.flags[mjVIS_CONTACTFORCE] = 0;
  }

  // if (wireframe)
  // {
  //   vopt.flags[mjVIS] = 1;
  // }
  // else
  // {
  //   vopt.flags[mjVIS_WIREFRAME] = 0;
  // }

  // Refresh UI display
  mjui_update(-1, -1, &ui0, &uistate, &con);
  mjui_update(-1, -1, &ui1, &uistate, &con);
}


// keyboard callback
void MujocoInitLoadObjects::keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods)
{
  getInstance().keyboardCBImpl(window, key, scancode, act, mods);
}

void MujocoInitLoadObjects::keyboardCBImpl([[maybe_unused]] GLFWwindow* window,
                                           int key,
                                           [[maybe_unused]] int scancode,
                                           int act,
                                           [[maybe_unused]] int mods)
{
  if (act == GLFW_PRESS)
  {
    if (mods & GLFW_MOD_SHIFT)
    {
      // Toggle UI panel with Shift key
      ui_visible = !ui_visible;
      std::cout << std::flush << (ui_visible ? "UI opened\n" : "UI closed\n") << std::endl;
    }
    else if (key == GLFW_KEY_SPACE)
    {
      paused = !paused;
      std::cout << (paused ? "Paused\n" : "Running\n");
    }
    else if (key == GLFW_KEY_ESCAPE)
    {
      glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
  }
}


// mouse button callback
void MujocoInitLoadObjects::mouseButtonCB(GLFWwindow* window, int button, int act, int mods)
{
  getInstance().mouseButtonCBImpl(window, button, act, mods);
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

  // Update button state
  uistate.left   = (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_PRESS);
  uistate.right  = (button == GLFW_MOUSE_BUTTON_RIGHT && act == GLFW_PRESS);
  uistate.middle = (button == GLFW_MOUSE_BUTTON_MIDDLE && act == GLFW_PRESS);

  // Update modifier keys
  uistate.shift   = (mods & GLFW_MOD_SHIFT);
  uistate.alt     = (mods & GLFW_MOD_ALT);
  uistate.control = (mods & GLFW_MOD_CONTROL);

  // Get mouse position
  double xpos, ypos;
  glfwGetCursorPos(window, &xpos, &ypos);
}


// mouse move callback
void MujocoInitLoadObjects::mouseMoveCB(GLFWwindow* window, double xpos, double ypos)
{
  getInstance().mouseMoveCBImpl(window, xpos, ypos);
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

  // determine act based on mouse button
  mjtMouse act;
  if (button_right)
  {
    act = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  }
  else if (button_left)
  {
    act = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  else
  {
    act = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, act, dx / height, dy / height, &scn, &cam);
}


// scroll callback
void MujocoInitLoadObjects::scrollCB(GLFWwindow* window, double xoffset, double yoffset)
{
  getInstance().scrollCBImpl(window, xoffset, yoffset);
}

void MujocoInitLoadObjects::scrollCBImpl([[maybe_unused]] GLFWwindow* window,
                                         [[maybe_unused]] double xoffset,
                                         double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void MujocoInitLoadObjects::render(GLFWwindow* window)
{
  // Get window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // Update right UI position
  uistate.rect[1].left = width - uistate.rect[1].width;

  // Render 3D scene
  mjrRect viewport = {0, 0, width, height};
  mjv_updateScene(m, d, &vopt, NULL, &cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);

  // Render UI panels on top
  if (ui0_enable)
  {
    mjr_rectangle(uistate.rect[0], 0.2, 0.2, 0.2, 0.7); // Background
    mjui_render(&ui0, &uistate, &con);
  }

  if (ui1_enable)
  {
    mjr_rectangle(uistate.rect[1], 0.2, 0.2, 0.2, 0.7); // Background
    mjui_render(&ui1, &uistate, &con);
  }

  // Swap buffers
  glfwSwapBuffers(window);
}

void MujocoInitLoadObjects::controlCB(const mjModel* m, mjData* d)
{
  getInstance().controlCBImpl(m, d);
}
void MujocoInitLoadObjects::controlCBImpl(const mjModel* m, mjData* d)
{
  // Check if controls are equal

  // for (int i = 0; i < dim; i++) {
  //     std::cout << d->sensordata[start + i] << std::endl;
  // }

  // better automated ways and not hardcoding
  for (int i = 0; i < 6; ++i)
  {
    auto test = command_buffer_->pop_value(command_type_, i, data_out);
    // std::cout<<std::flush<<"test: "<<test<<std::endl;
    if (test)
    {
      d->ctrl[i] = data_out;
      // std::cout<<std::flush<<std::endl<<data_out;
    }

    state_buffer_->push_value(
      CommandTypes::POSITION, i, d->qpos[m->jnt_qposadr[mujoco_joint_ids_[i]]]);
    state_buffer_->push_value(
      CommandTypes::VELOCITY, i, d->qvel[m->jnt_qposadr[mujoco_joint_ids_[i]]]);
    state_buffer_->push_value(
      CommandTypes::EFFORT, i, d->qacc[m->jnt_qposadr[mujoco_joint_ids_[i]]]);
  }

  // better automated ways and not hardcoding
  for (int i = 0; i < mujoco_sensor_ids_.size(); i++)
  {
    sensor_buffer_->push_value(CommandTypes::SENSOR, i, d->sensordata[mujoco_sensor_ids_[i]]);
  }
}
void MujocoInitLoadObjects::init(mjModel* mujoco_model, mjData* mujoco_data)
{
  return getInstance().initialize_simulation(mujoco_model, mujoco_data);
}
void MujocoInitLoadObjects::initialize_simulation(mjModel* mujoco_model, mjData* mujoco_data)
{
  // (Test) Load XML manually for now and test the model
  try
  {
    m = mujoco_model;
    d = mujoco_data;

    std::cout << std::flush << "Simulation Initialized" << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
}

void MujocoInitLoadObjects::start_simulation(bool single_thread)
{
  std::cout << std::flush << "Starting Simulation" << std::endl;
  getInstance().starting_simulation(single_thread);
}

void MujocoInitLoadObjects::starting_simulation(bool single_thread)
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
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // UI init
    std::memset(&uistate, 0, sizeof uistate);
    std::memset(&ui0, 0, sizeof ui0);
    ui0.spacing = mjui_themeSpacing(0);
    ui0.color   = mjui_themeColor(0);
    ui0.rectid  = 1; // <- will draw into uistate.rect[1]
    ui0.auxid   = 0;

    static mjuiDef def_panel[] = {{mjITEM_SECTION, "Simulation", 1, nullptr, "AS"},
                                  {mjITEM_BUTTON, "Pause/Run (Space)", 2},
                                  {mjITEM_END}};
    mjui_add(&ui0, def_panel);
    mjui_resize(&ui0, &con); // compute panel width, etc.


    glfwSetKeyCallback(window, keyboardCB);
    glfwSetMouseButtonCallback(window, mouseButtonCB);
    glfwSetCursorPosCallback(window, mouseMoveCB);
    glfwSetScrollCallback(window, scrollCB);


    mjcb_control = MujocoInitLoadObjects::controlCB;

    // ... install GLFW keyboard and mouse callbacks

    // run main loop, target real-time simulation and 60 fps rendering

    bool simulation_start = false;
    while (!glfwWindowShouldClose(window))
    {
      glfwPollEvents();
      // advance interactive simulation for 1/60 sec
      //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
      //  this loop will finish on time for the next frame to be rendered at 60 fps.
      //  Otherwise add a cpu timer and exit this loop when it is time to render.
      mjtNum simstart = d->time;
      while (d->time - simstart < 1.0 / 60.0)
      {
        mj_step(m, d);
      }


      // update scene and render
      mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

      // Render
      int width, height;
      glfwGetFramebufferSize(window, &width, &height);

      uistate.nrect = 2;

      // main viewport
      uistate.rect[0].left   = 0;
      uistate.rect[0].bottom = 0;
      uistate.rect[0].width  = width;
      uistate.rect[0].height = height;

      // right panel (ui0.rectid == 1 -> uistate.rect[1])
      int panel_w            = ui_visible ? ui0.width : 0;
      uistate.rect[1].width  = panel_w;
      uistate.rect[1].left   = (panel_w > 0) ? width - panel_w : width; // stick to right edge
      uistate.rect[1].bottom = 0;
      uistate.rect[1].height = height;

      mjrRect viewport = {0, 0, width, height};
      mjr_render(viewport, &scn, &con);

      if (ui_visible)
      {
        mjr_rectangle(uistate.rect[1], 0.2f, 0.2f, 0.2f, 0.7f);
        mjui_render(&ui0, &uistate, &con); // draw UI; events already handled in callbacks
      }

      // swap OpenGL buffers (blocking call due to v-sync)
      glfwSwapBuffers(window);

      // process pending GUI events, call GLFW callbacks

      if (!simulation_start & !single_thread)
      {
        std::cout << std::flush << "Unlocking and notifying" << std::endl;
        simulation_start = true;
        processed        = true;
        lk.unlock();
        cv.notify_one();
      }
    }
  }
  DeleteData();
}
void MujocoInitLoadObjects::DeleteData()
{
  std::cout << std::flush << "Cleaning up Simulation Data" << std::endl;
  getInstance().DeletingData();
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
