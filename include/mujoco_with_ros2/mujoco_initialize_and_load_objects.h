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
#  include <mujoco/mjui.h>
#  include <mujoco/mjvisualize.h>

#  include <mujoco_with_ros2/command_and_state_buffer.h>
#  include <realtime_tools/lock_free_queue.hpp>

extern std::mutex mut_ready;
extern std::condition_variable cv;
extern bool ready;
extern bool processed;

namespace mujoco_with_ros2 {

// Define UI Section IDs
enum
{
  // Left UI sections
  SECT_FILE = 0,
  SECT_SIMULATION,
  SECT_RENDERING,
  SECT_VISUALIZATION,
  NSECT0 // Total number of left UI sections
};

enum
{
  // Right UI sections
  SECT_JOINT = 0,
  SECT_CONTROL,
  NSECT1 // Total number of right UI sections
};

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
  mjUI ui0;            // Left UI panel
  mjUI ui1;            // Right UI panel
  mjuiState uistate;   // UI interaction state
  mjvOption vopt;

  // UI control variables (these were global)
  int paused       = 0;
  int speed        = 1;
  int show_contact = 1;
  int show_forces  = 0;
  int frame_rate   = 60;
  int wireframe    = 0;
  int transparent  = 0;
  int ui0_enable;
  int ui1_enable;

  // UI section definitions (can be static const)
  static const mjuiDef defFile[];
  static const mjuiDef defSimulation[];
  static const mjuiDef defRendering[];

  void initUI()
  { 
    
    std::cout << std::flush << "Initializing UI : " << std::endl;
    // Left UI FIles
    static const mjuiDef defFile[] = {{mjITEM_SECTION, "File", mjPRESERVE, NULL, "AF"},
                                      {mjITEM_BUTTON, "Save XML", 2, NULL, ""},
                                      {mjITEM_BUTTON, "Save Model", 2, NULL, ""},
                                      {mjITEM_BUTTON, "Reset", 2, NULL, " "},
                                      {mjITEM_BUTTON, "Quit", 2, NULL, "Escape"},
                                      {mjITEM_END}};
    std::cout << std::flush << "Initialized FILE : " << std::endl;
    static const mjuiDef defSimulation[] = {{mjITEM_SECTION, "Simulation", mjPRESERVE, NULL, "AS"},
                                            {mjITEM_CHECKINT, "Paused", 2, &paused, " "},
                                            {mjITEM_SLIDERINT, "Speed", 2, &speed, "1 10"},
                                            {mjITEM_BUTTON, "Reset", 2, NULL, "Backspace"},
                                            {mjITEM_END}};
    std::cout << std::flush << "Initialized Simulation : " << std::endl;
    static const mjuiDef defVisualization[] = {
      {mjITEM_SECTION, "Visualization", mjPRESERVE, NULL, "AV"},
      {mjITEM_CHECKINT, "Wireframe", 2, &wireframe, "W"},
      {mjITEM_CHECKINT, "Transparent", 2, &transparent, "T"},
      {mjITEM_END}};
    std::cout << std::flush << "Initialized Visualization : " << std::endl;
    static const mjuiDef defRendering[] = {
      {mjITEM_SECTION, "Rendering", mjPRESERVE, NULL, "AR"},
      {mjITEM_CHECKINT, "Show Contact", 2, &show_contact, "C"},
      {mjITEM_CHECKINT, "Show Forces", 2, &show_forces, "F"},
      {mjITEM_SLIDERINT, "Frame Rate", 2, &frame_rate, "30 120"},
      {mjITEM_END}};
    std::cout << std::flush << "Initialized rendering : " << std::endl;

    // Clear UI structures
    memset(&ui0, 0, sizeof(mjUI));
    memset(&ui1, 0, sizeof(mjUI));
    memset(&uistate, 0, sizeof(mjuiState));
    std::cout << std::flush << "cleared memory " << std::endl;
    // Set UI theme (optional)
    uistate.nrect          = 2; // Number of UI panels
    uistate.rect[0].left   = 0;
    uistate.rect[0].bottom = 0;
    uistate.rect[0].width  = 250; // Left panel width

    uistate.rect[1].left   = 0; // Will be set to window_width - 250
    uistate.rect[1].bottom = 0;
    uistate.rect[1].width  = 250; // Right panel width
    
    std::cout << std::flush << "set value " << std::endl;
    // Add sections to LEFT UI (ui0)
    mjui_add(&ui0, defFile);
    std::cout << std::flush << "problem 1 " << std::endl;
    mjui_add(&ui0, defSimulation);
    std::cout << std::flush << "problem 2 " << std::endl;
    mjui_add(&ui0, defRendering);
    std::cout << std::flush << "problem 3 " << std::endl;
    mjui_add(&ui0, defVisualization);
    std::cout << std::flush << "problem 4 " << std::endl;

    std::cout << std::flush << "Initialized left UI : " << std::endl;
    // Initialize RIGHT UI (ui1) - for joints and controls
    initJointControlUI();
  }

  void initJointControlUI()
  {
    std::cout << std::flush << "Initializing right UI : " << std::endl;
    if (!m)
      return;

    // Add joint section header
    const static mjuiDef defJoint = {mjITEM_SECTION, "Joints", mjPRESERVE, NULL, "AJ"};
    mjui_add(&ui1, &defJoint);

    // Add slider for each joint
    for (int i = 0; i < m->njnt; i++)
    {
      int qposadr      = m->jnt_qposadr[i];
      const char* name = mj_id2name(m, mjOBJ_JOINT, i);

      mjuiDef def;
      def.type = mjITEM_SLIDERNUM;
      mju_strncpy(def.name, name ? name : "joint", mjMAXUINAME);
      def.state = 2;
      def.pdata = &d->qpos[qposadr];
      mju_strncpy(def.other, "-3.14 3.14", mjMAXUITEXT); // Range

      mjui_add(&ui1, &def);
    }

    // Add control section header
    const static mjuiDef defControl = {mjITEM_SECTION, "Controls", mjPRESERVE, NULL, "AC"};
    mjui_add(&ui1, &defControl);

    // Add slider for each actuator
    for (int i = 0; i < m->nu; i++)
    {
      const char* name = mj_id2name(m, mjOBJ_ACTUATOR, i);

      mjuiDef def;
      def.type = mjITEM_SLIDERNUM;
      mju_strncpy(def.name, name ? name : "ctrl", mjMAXUINAME);
      def.state = 2;
      def.pdata = &d->ctrl[i];
      mju_strncpy(def.other, "-1 1", mjMAXUITEXT); // Range

      mjui_add(&ui1, &def);
    }

    // Finalize - must be called after adding all items
    mjuiDef defEnd = {mjITEM_END};
    mjui_add(&ui1, &defEnd);
  }

  void updateUI();
  void uiEvent(mjuiItem* item);
  void render(GLFWwindow* window);

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
  std::vector<int> mujoco_sensor_ids_;
  CommandTypes command_type_;
  // Joint Names
  std::vector<std::string> ur5e_joint_names;

  // buffers
  std::shared_ptr<CommandBuffer> command_buffer_;
  std::shared_ptr<StateBuffer> state_buffer_;
  std::shared_ptr<StateBuffer> sensor_buffer_;

  //(To Review maybe a better way (Singleton Class))
  // static Init method to return an static instance of initialized simulation (static because
  // otherwise the method doesn't point from an object and is dangling, static so it can be called
  // and persists and initiliaze zthe class)
  static void init(mjModel* mujoco_model, mjData* mujoco_data);
  // Initialize the Simulation and load objects
  void initialize_simulation(mjModel* mujoco_model, mjData* mujoco_data);

  // Start Simulation loop and launch the rendering window
  static void start_simulation(bool single_thread = false);
  void starting_simulation(bool single_thread = false);

  // Keyboard callback
  static void keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods);
  void keyboardCBImpl(GLFWwindow* window, int key, int scancode, int act, int mods);

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

  // provide shared buffers for connections
  void initialize_buffers(std::shared_ptr<CommandBuffer> c_buff,
                          std::shared_ptr<StateBuffer> s_buff,
                          std::shared_ptr<StateBuffer> sens_buff,
                          std::vector<int> joint_ids_,
                          std::vector<int> sensor_ids_);
};


} // namespace mujoco_with_ros2

#endif
