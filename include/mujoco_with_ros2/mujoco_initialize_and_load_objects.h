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
#ifndef MUJOCO_LOAD_AND_INITIALIZE_OBJECTS_H
#  define MUJOCO_LOAD_AND_INITIALIZE_OBJECTS_H

#  include <array>
#  include <atomic>
#  include <condition_variable>
#  include <cstdio>
#  include <cstring>
#  include <memory>
#  include <mutex>
#  include <string>
#  include <thread>
#  include <vector>

#  include "GLFW/glfw3.h"
#  include "mujoco/mujoco.h"

#  include <mujoco_with_ros2/command_and_state_buffer.h>
#  include <realtime_tools/lock_free_queue.hpp>

extern std::mutex mut_ready;
extern std::condition_variable cv;
extern bool ready;
extern bool processed;

namespace mujoco_with_ros2 {

class MujocoInitLoadObjects
{
private:
  MujocoInitLoadObjects();
  ~MujocoInitLoadObjects()
  {
    std::cout << std::endl << std::flush << "Destroying the simulation object";
  }

public:
  static MujocoInitLoadObjects& getInstance()
  {
    static MujocoInitLoadObjects load_objects_simulation;
    return load_objects_simulation;
  }

  MujocoInitLoadObjects(const MujocoInitLoadObjects&)            = delete;
  MujocoInitLoadObjects& operator=(const MujocoInitLoadObjects&) = delete;
  MujocoInitLoadObjects(MujocoInitLoadObjects&&)                 = delete;
  MujocoInitLoadObjects& operator=(MujocoInitLoadObjects&&)      = delete;

  // MuJoCo data structures
  mjSpec* spec = NULL; // MuJoCo Spec
  mjModel* m   = NULL; // MuJoCo model
  mjData* d    = NULL; // MuJoCo data
  mjvCamera cam;       // abstract camera
  mjvOption opt;       // visualization options
  mjvScene scn;        // abstract scene
  mjrContext con;      // custom GPU context
  double data_out;

  // mouse interaction
  bool button_left   = false;
  bool button_middle = false;
  bool button_right  = false;
  double lastx       = 0;
  double lasty       = 0;

  // multi threading requirements
  bool is_deleted = false;

  // Buffers for interaction with ROS2 Hardware_interface
  std::vector<int> mujoco_joint_ids_;

  // Joint Names
  std::vector<std::string> ur5e_joint_names;

  // buffers
  std::shared_ptr<CommandBuffer> command_buffer_;
  std::shared_ptr<StateBuffer> state_buffer_;
  //(To Review maybe a better way (Singleton Class))
  // static Init method to return an static instance of initialized simulation (static because
  // otherwise the method doesn't point from an object and is dangling, static so it can be called
  // and persists and initiliaze zthe class)
  static void init(mjModel* mujoco_model,mjData* mujoco_data);
  // Initialize the Simulation and load objects
  void initialize_simulation(mjModel* mujoco_model,mjData* mujoco_data);

  // Start Simulation loop and launch the rendering window
  static void start_simulation(bool single_thread = false);
  void starting_simulation(bool single_thread = false);

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

  //provide shared buffers for connections
  void initialize_buffers(std::shared_ptr<CommandBuffer> c_buff,
                                               std::shared_ptr<StateBuffer> s_buff,std::vector<int> joint_ids_);
};


} // namespace mujoco_with_ros2

#endif
