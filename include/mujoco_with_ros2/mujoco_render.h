#ifndef MUJOCO_RENDER_H_
#define MUJOCO_RENDER_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <ratio>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "mujoco_with_ros2/mujoco_with_ros2_ui_adapter.h"


using Seconds      = std::chrono::duration<double>;
using Milliseconds = std::chrono::duration<double, std::milli>;

namespace mujoco_with_ros2 {

class SimulateMutex : public std::recursive_mutex
{
};
using MutexLock = std::unique_lock<std::recursive_mutex>;

class MujocoRender
{
private:
public:
  // Clock and timer Handling
  using Clock = std::chrono::steady_clock;
static_assert( (Clock::period::num * 1'000LL) <= Clock::period::den,
               "steady_clock must have ≤1ms resolution");  

 MujocoRender(std::unique_ptr<MujocoWithRos2UIAdapter> mujoco_with_ros2_adapter,mjvCamera* cam,mjvOption* opt,mjvPerturb* pert);
 
   // functions below are used by the renderthread
// load mjb or xml model that has been requested by load()
 void LoadOnRenderThread();
 // render the ui to the window
  void Render();
  // loop to render the UI (must be called from main thread because of MacOS)
  void RenderLoop();



// abstract visualization and model/data
  mjvScene scn;
  mjvCamera& cam;
  mjvOption& opt;
  mjvPerturb& pert;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  mjModel* mnew_ = nullptr;
  mjData* dnew_ = nullptr;

//Utility
  int ncam_ = 0;
  int nkey_ = 0;
  int state_size_ = 0;      // number of mjtNums in a history buffer state
  int nhistory_ = 0;        // number of states saved in history buffer
  int history_cursor_ = 0;  // cursor pointing at last saved state
  std::vector<mjtNum> history_;  // history buffer (nhistory x state_size)
  // index of history-scrubber slider
  int scrub_index = 0;

  std::vector<int> body_parentid_;
  std::vector<int> jnt_type_;
  std::vector<int> jnt_group_;
  std::vector<int> jnt_qposadr_;
  std::vector<std::string> jnt_names_;
  std::vector<int> actuator_group_;
  std::vector<std::string> actuator_names_;

  // mjModel and mjData fields that can be modified by the user through the GUI
  std::vector<mjtNum> qpos_;
  std::vector<mjtNum> qpos_prev_;
  std::vector<mjtNum> ctrl_;
  std::vector<mjtNum> ctrl_prev_;

// OpenGL rendering and UI
  int refresh_rate = 60;
  int window_pos[2] = {0};
  int window_size[2] = {0};
  std::unique_ptr<MujocoWithRos2UIAdapter> mujoco_with_ros2_ui;
  mjuiState& uistate;
  mjUI ui0 = {};
  mjUI ui1 = {};

  //options
  int spacing = 0;
  int color = 0;
  int font = 0;
  int help = 0;
  int info = 0;
  int pause_update = 0;
  int fullscreen = 0;
  int vsync = 1;
  int ui0_enable = 1;
  int ui1_enable = 1;

  //display vsync options
  int frames_ = 0;
  std::chrono::time_point<Clock> last_fps_update_;
  double fps_ = 0;

  //constants and maximum geometeries possible 
  static constexpr int kMaxGeom = 100000;

  //UI SECTIONS ARRAYS
    const mjuiDef def_option[13] = {
    {mjITEM_SECTION,  "Option",        mjPRESERVE, nullptr,  "AO"},
    {mjITEM_CHECKINT, "Help",          2, &this->help,       " #290"},
    {mjITEM_CHECKINT, "Info",          2, &this->info,       " #291"},
    {mjITEM_CHECKINT, "Pause update",  2, &this->pause_update,    ""},
    {mjITEM_CHECKINT, "Fullscreen",    1, &this->fullscreen, " #294"},
    {mjITEM_END}
  };

  //multi threading requirements
  SimulateMutex mtx;
  std::condition_variable_any cond_loadrequest;

  // loadrequest
  //   3: display a loading message
  //   2: render thread asked to update its model
  //   1: showing "loading" label, about to load
  //   0: model loaded or no load requested.
  int loadrequest = 0;
};
} // namespace mujoco_with_ros2


#endif
