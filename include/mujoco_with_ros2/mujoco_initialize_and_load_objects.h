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

#pragma once

#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

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

  //(To Review maybe a better way (Singleton Class))
  // static Init method to return an static instance of initialized simulation (static because
  // otherwise the method doesn't point from an object and is dangling, static so it can be called
  // and persists and initiliaze the class)
  static void init();
  // Initialize the Simulation and load objects
  void initialize_simulation();
};


} // namespace mujoco_with_ros2
