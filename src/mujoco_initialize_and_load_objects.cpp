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

void MujocoInitLoadObjects::init()
{
  std::cout << std::flush << "Simulation Initialized" << std::endl;
  getInstance().initialize_simulation();
}
void MujocoInitLoadObjects::initialize_simulation()
{
  // (Test) Load XML manually for now and test the model
  try
  {
    char err_str[1000];
    int err_str_sz = 1000;
    mjModel* m = mj_loadXML(
      "/home/saksham/checkout/thesis_ws/colcon_ws/src/mujoco_with_ros2/models/ur5e/urdf/ur5e.xml",
      NULL,
      err_str,
      err_str_sz);
    if (!m)
    {
      std::cout << std::flush << "Problem with model" << std::endl;
      std::cout << std::flush << err_str << std::endl;
    }

    mjData* d = mj_makeData(m);
    
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context
    if (!glfwInit())
    {
      mju_error("Could not initialize GLFW");
    }
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 1000);
  mjr_makeContext(m, &con, mjFONTSCALE_100);

  // ... install GLFW keyboard and mouse callbacks

  // run main loop, target real-time simulation and 60 fps rendering
  while( !glfwWindowShouldClose(window) ) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    while( d->time - simstart < 1.0/60.0 )
        mj_step(m, d);

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

  // close GLFW, free visualization storage
  glfwTerminate();
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

};
} // namespace mujoco_with_ros2

// Should be a global main so that the linker finds it, inside the namespace it represents that
// namespace
int main()
{
  std::cout << std::flush << "Staring Simulation" << std::endl;
  // call the static function here
  mujoco_with_ros2::MujocoInitLoadObjects::init();
  return 0;
}
