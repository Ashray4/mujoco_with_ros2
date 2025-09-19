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
#  include <iostream>
#  include <memory>
#  include <mutex>
#  include <string>
#  include <thread>
#  include <vector>

#  include "GLFW/glfw3.h"
#  include "mujoco/mujoco.h"

#  include <realtime_tools/lock_free_queue.hpp>

extern std::mutex mut_ready;
extern std::condition_variable cv;
extern bool ready;
extern bool processed;

namespace mujoco_with_ros2 {
// static mujoco_with_ros2::MujocoInitLoadObjects* global_mujoco_instance = nullptr;
class MujocoInitLoadObjects
{
  using QueueType = realtime_tools::LockFreeQueueBase<double, boost::lockfree::spsc_queue<double> >;
  using QueuePtr  = std::unique_ptr<QueueType>;

private:
  static std::atomic_bool alive_;

public:
  MujocoInitLoadObjects(std::vector<QueuePtr>& joint_commands_,
                        std::vector<QueuePtr>& joint_pos_states_,
                        std::vector<QueuePtr>& joint_vel_states_,
                        std::vector<QueuePtr>& joint_eff_states_,
                        const std::vector<int>& joint_ids_,
                        bool single_thread);
  ~MujocoInitLoadObjects();

  // MuJoCo data structures
  mjSpec* spec = NULL; // MuJoCo Spec
  mjModel* m   = NULL; // MuJoCo model
  mjData* d    = NULL; // MuJoCo data
  mjvCamera cam;       // abstract camera
  mjvOption opt;       // visualization options
  mjvScene scn;        // abstract scene
  mjrContext con;      // custom GPU context
  GLFWwindow* window_;

  // mouse interaction
  bool button_left   = false;
  bool button_middle = false;
  bool button_right  = false;
  double lastx       = 0;
  double lasty       = 0;

  // multi threading requirements
  bool is_deleted = false;
  bool single_thread;

  // Buffers for interaction with ROS2 Hardware_interface
  // Joint states
  std::vector<double> joint_positions_state;
  std::vector<double> joint_velocity_state;
  std::vector<double> joint_acceleration_state;
  std::vector<double> time_state;
  // Joint Inputs
  std::vector<double> joint_positions_input;
  std::vector<double> joint_velocity_input;
  std::vector<double> joint_acceleration_input;

  std::vector<QueuePtr>& joint_commands_;
  std::vector<QueuePtr>& joint_pos_states_;
  std::vector<QueuePtr>& joint_vel_states_;
  std::vector<QueuePtr>& joint_eff_states_;

  // Joint Names and ids
  std::vector<std::string> ur5e_joint_names;
  std::vector<int> ur5e_joint_ids;

  //(To Review maybe a better way (Singleton Class))
  // static Init method to return an static instance of initialized simulation (static because
  // otherwise the method doesn't point from an object and is dangling, static so it can be called
  // and persists and initiliaze the class)
  // Initialize the Simulation and load objects
  mjModel* initialize_simulation();

  // Start Simulation loop and launch the rendering window

  void starting_simulation();

  // Keyboard callback
  // static void keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods);
  // void keyboardCBImpl(GLFWwindow* window, int key, int scancode, int act, int mods);

  // Mouse button callback
  void addCallbacks();

  static void mouseButtonCB(GLFWwindow* window, int button, int act, int mods);
  void mouseButtonCBImpl(GLFWwindow* window, int button, int act, int mods);
  
  //
  
  static void mouseMoveCB(GLFWwindow* window, double xpos, double ypos);
  void mouseMoveCBImpl(GLFWwindow* window, double xpos, double ypos);
  
  static void scrollCB(GLFWwindow* window, double xoffset, double yoffset);
  void scrollCBImpl(GLFWwindow* window, double xoffset, double yoffset);
  
  static void controlCB(const mjModel* m, mjData* d);
  void controlCBImpl(const mjModel* m, mjData* d);

  void DeletingData();

  MujocoInitLoadObjects* loaded_object_instance_;

};


} // namespace mujoco_with_ros2

#endif

// #define REP10(P, M)  M(P##0) M(P##1) M(P##2) M(P##3) M(P##4) M(P##5) M(P##6) M(P##7) M(P##8) M(P##9)
// #define REP100(M) REP10(,M) REP10(1,M) REP10(2,M) REP10(3,M) REP10(4,M) REP10(5,M) REP10(6,M) REP10(7,M) REP10(8,M) REP10(9,M)

// typedef void (*callback_fn_t)(void);  // or whatever signature you need

// class myclass {
//     static struct callback_t {
//         callback_t      *next;
//         callback_fn_t   callback;
//         myclass         *obj;
//     } callback_table[100];
//     callback_t          *my_callback;
//     static callback_t   *freelist;
// #define CB_FUNC_DECL(M)  static void cbfunc##M() { callback_table[M].obj->callback(); }
//     REP100(CB_FUNC_DECL)

//  public:
//     callback_fn_t get_callback() {
//         if (!my_callback) {
//             if (!freelist) return nullptr;
//             my_callback = freelist;
//             freelist = my_callback->next;
//             my_callback->obj = this; }
//         return my_callback->callback;
//     }
//     void callback() {
//         /* this non-static method is called by the callback */
//     }

//     myclass() : my_callback(nullptr) { }
//     myclass(const myclass &a) : my_callback(nullptr) {
//         // need to manually define copy
//     }
//     ~myclass() {
//         if (my_callback) { 
//             my_callback->obj = nullptr;
//             my_callback->next = freelist;
//             freelist = my_callback; }
//     }
// };

// #define CB_TABLE_INIT(M) { M ? myclass::callback_table+M-1 : 0, myclass::cbfunc##M },
// myclass::callback_t myclass::callback_table[100] = { REP100(CB_TABLE_INIT) };
// myclass::callback_t *myclass::freelist = &myclass::callback_table[99];
