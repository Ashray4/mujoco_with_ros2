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

namespace mujoco_with_ros2 {

MujocoInitLoadObjects::MujocoInitLoadObjects() {}

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

void MujocoInitLoadObjects::controlCB(const mjModel* m, mjData* d)
{
  getInstance().controlCBImpl(m, d);
}
void MujocoInitLoadObjects::controlCBImpl(const mjModel* m, mjData* d)
{ 
  
  // Check if controls are equal
   for (int i = 0; i < m->nq; ++i)
        {
            d->ctrl[i] = 1.0;                                                                                                                              
        }
}
mjModel* MujocoInitLoadObjects::init()
{
  return getInstance().initialize_simulation();
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

    m = mj_compile(spec, NULL);
    d = mj_makeData(m);

    std::cout << std::flush << "Simulation Initialized" << std::endl;
    return m;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return nullptr;
  }
}

void MujocoInitLoadObjects::start_simulation()
{
  std::cout << std::flush << "Starting Simulation" << std::endl;
  getInstance().starting_simulation();
}

void MujocoInitLoadObjects::starting_simulation()
{ //check if valid model available
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


    // install GLFW mouse and keyboard callbacks
    // glfwSetKeyCallback(window, keyboardCB);
    glfwSetMouseButtonCallback(window, mouseButtonCB);
    glfwSetCursorPosCallback(window, mouseMoveCB);
    glfwSetScrollCallback(window, scrollCB);


    mjcb_control = MujocoInitLoadObjects::controlCB;

    // ... install GLFW keyboard and mouse callbacks

    // run main loop, target real-time simulation and 60 fps rendering
    while (!glfwWindowShouldClose(window))
    {
      // advance interactive simulation for 1/60 sec
      //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
      //  this loop will finish on time for the next frame to be rendered at 60 fps.
      //  Otherwise add a cpu timer and exit this loop when it is time to render.
      mjtNum simstart = d->time;
      while (d->time - simstart < 1.0 / 60.0)
      {
        mj_step(m, d);
      }

      // get framebuffer viewport
      mjrRect viewport = {0, 0, 0, 0};
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

      // update scene and render
      mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
      mjr_render(viewport, &scn, &con);

      // swap OpenGL buffers (blocking call due to v-sync)
      glfwSwapBuffers(window);

      // process pending GUI events, call GLFW callbacks
      glfwPollEvents();
    }
  }
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
}
} // namespace mujoco_with_ros2

// Should be a global main so that the linker finds it, inside the namespace it represents that
// namespace
int main()
{
  // call the static function here
  mujoco_with_ros2::MujocoInitLoadObjects::init();

  // start Simulation and visualization
  mujoco_with_ros2::MujocoInitLoadObjects::start_simulation();

  // delete data
  mujoco_with_ros2::MujocoInitLoadObjects::DeleteData();

  return 0;
}
