// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    mujoco_initialize_and_load_objects.h
 *
 * \author  Saksham Kohli <kohli@rptu.de>
 * \date    2025/09/10
 *
 */
//-----------------------------------------------------------------------------

// The Purpose of this script is to load the objects,robots and other sensors into the simulation
// and visualize them, for objects where a publisher or a ros2 controller is needed are initialized
// and setup

#pragma once

#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <thread>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

#include <rclcpp/rclcpp.hpp>
namespace mujoco_with_ros2 {

class MujocoInitLoadObjects
{
private:
  MujocoInitLoadObjects();
  static MujocoInitLoadObjects& getInstance()
  {
    static MujocoInitLoadObjects load_objects_simulation;
    return load_objects_simulation;
  }

public:
  MujocoInitLoadObjects(const MujocoInitLoadObjects&)            = delete;
  MujocoInitLoadObjects& operator=(const MujocoInitLoadObjects&) = delete;
  MujocoInitLoadObjects(MujocoInitLoadObjects&&)                 = delete;
  MujocoInitLoadObjects& operator=(MujocoInitLoadObjects&&)      = delete;

  // MuJoCo data structures
  mjSpec* spec = NULL; // MuJoCo Spec
  mjModel* m = NULL; // MuJoCo model
  mjData* d  = NULL; // MuJoCo data
  mjvCamera cam;     // abstract camera
  mjvOption opt;     // visualization options
  mjvScene scn;      // abstract scene
  mjrContext con;    // custom GPU context


  // mouse interaction
  bool button_left   = false;
  bool button_middle = false;
  bool button_right  = false;
  double lastx       = 0;
  double lasty       = 0;

  //Buffers for interaction with ROS2 Hardware_interface_Interaction
  //Joint states
  std::vector<double> joint_positions_state;
  std::vector<double> joint_velocity_state;
  std::vector<double> joint_acceleration_state;
  
  //Joint Inputs
  std::vector<double> joint_positions_input;
  std::vector<double> joint_velocity_input;
  std::vector<double> joint_acceleration_input;
  
  //(To Review maybe a better way (Singleton Class))
  // static Init method to return an static instance of initialized simulation (static because
  // otherwise the method doesn't point from an object and is dangling, static so it can be called
  // and persists and initiliaze the class)
  static mjModel* init();
  // Initialize the Simulation and load objects
  mjModel* initialize_simulation();

  //Start Simulation loop and launch the rendering window
  static void start_simulation();
  void starting_simulation();

  // Keyboard callback
  // static void keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods);
  // void keyboardCBImpl(GLFWwindow* window, int key, int scancode, int act, int mods);

  // Mouse button callback
  static void mouseButtonCB(GLFWwindow* window, int button, int act, int mods);
  void mouseButtonCBImpl(GLFWwindow* window, int button, int act, int mods);
  //
  //// Mouse move callback
  static void mouseMoveCB(GLFWwindow* window, double xpos, double ypos);
  void mouseMoveCBImpl(GLFWwindow* window, double xpos, double ypos);

  // Scroll callback
  static void scrollCB(GLFWwindow* window, double xoffset, double yoffset);
  void scrollCBImpl(GLFWwindow* window, double xoffset, double yoffset);

  // Control input callback for the solver
  static void controlCB(const mjModel* m, mjData* d);
  void controlCBImpl(const mjModel* m, mjData* d);

  // Delete instance and cleanup 
  static void DeleteData();
  void DeletingData();
};


} // namespace mujoco_with_ros2
