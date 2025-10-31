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

void MujocoInitLoadObjects::uiEvent(mjuiItem* item)
{
  if (!item)
    return;

  // Handle buttons by name
  if (strcmp(item->name, "Save XML") == 0)
  {
    mj_saveLastXML("saved_model.xml", m, NULL, 0);
    printf("Model saved to saved_model.xml\n");
  }
  else if (strcmp(item->name, "Save Model") == 0)
  {
    mj_saveModel(m, "saved_model.mjb", NULL, 0);
    printf("Model saved to saved_model.mjb\n");
  }
  else if (strcmp(item->name, "Reset") == 0)
  {
    mj_resetData(m, d);
    printf("Simulation reset\n");
  }
  else if (strcmp(item->name, "Quit") == 0)
  {
    glfwSetWindowShouldClose(glfwGetCurrentContext(), 1);
  }
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
  // Only process key press and repeat, not release
  if (act == GLFW_RELEASE)
  {
    return;
  }

  // Update modifier key state in UI
  uistate.shift   = (mods & GLFW_MOD_SHIFT);
  uistate.alt     = (mods & GLFW_MOD_ALT);
  uistate.control = (mods & GLFW_MOD_CONTROL);

  // Let UI handle event FIRST if mouse is over UI
  mjuiItem* item = mjui_event(&ui0, &uistate, &con);
  if (!item)
  {
    item = mjui_event(&ui1, &uistate, &con);
  }

  // If UI handled the event (button clicked, etc.)
  if (item)
  {
    uiEvent(item);
    return; // UI consumed the event
  }


  // --------------------------------------------------------------------
  // Handle UI visibility toggles
  // --------------------------------------------------------------------
  if (key == GLFW_KEY_TAB)
  {
    if (uistate.shift)
    {
      // Shift+Tab: Toggle RIGHT UI
      ui1_enable = !ui1_enable;
    }
    else
    {
      // Tab: Toggle LEFT UI
      ui0_enable = !ui0_enable;
    }
    return;
  }

  // --------------------------------------------------------------------
  // Handle UI section expand/collapse
  // Note: This is typically done with double-click, but you can add
  // keyboard shortcuts here if desired
  // --------------------------------------------------------------------

  // --------------------------------------------------------------------
  // Your existing keyboard shortcuts
  // --------------------------------------------------------------------
  switch (key)
  {
    case GLFW_KEY_SPACE:
      paused = !paused;
      break;

    case GLFW_KEY_BACKSPACE:
      mj_resetData(m, d);
      break;

    case GLFW_KEY_ESCAPE:
      glfwSetWindowShouldClose(window, 1);
      break;

    case GLFW_KEY_C:
      show_contact = !show_contact;
      break;

    case GLFW_KEY_F:
      show_forces = !show_forces;
      break;

    case GLFW_KEY_W:
      wireframe = !wireframe;
      break;

    case GLFW_KEY_T:
      transparent = !transparent;
      break;

      // Add more shortcuts as needed
  }

  updateUI();
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

  // Update UI mouse position
  uistate.x = (int)xpos;
  uistate.y = (int)ypos;

  // Let UI handle the event
  mjuiItem* item = mjui_event(&ui0, &uistate, &con);
  if (!item)
  {
    item = mjui_event(&ui1, &uistate, &con);
  }

  // Handle UI button clicks
  if (item)
  {
    uiEvent(item);
  }
}


// mouse move callback
void MujocoInitLoadObjects::mouseMoveCB(GLFWwindow* window, double xpos, double ypos)
{
  getInstance().mouseMoveCBImpl(window, xpos, ypos);
}

void MujocoInitLoadObjects::mouseMoveCBImpl(GLFWwindow* window, double xpos, double ypos)
{
  // Update UI mouse position
  uistate.x = (int)xpos;
  uistate.y = (int)ypos;

  // Let UI handle the event (for dragging sliders, etc.)
  mjui_event(&ui0, &uistate, &con);
  mjui_event(&ui1, &uistate, &con);


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
  // Update scroll state
  uistate.sy = (int)yoffset;

  // Let UI handle scroll (for scrolling long panels)
  mjui_event(&ui0, &uistate, &con);
  mjui_event(&ui1, &uistate, &con);

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

    std::cout << "6. Init UI structures" << std::endl;
    memset(&ui0, 0, sizeof(mjUI));
    memset(&uistate, 0, sizeof(mjuiState));

    std::cout << "7. Create UI definition" << std::endl;
    static const mjuiDef defFile[] = {{mjITEM_SECTION, "File", mjPRESERVE, NULL, "AF"},
                                      {mjITEM_BUTTON, "Quit", 2, NULL, ""},
                                      {mjITEM_END}};

    std::cout << "8. Call mjui_add" << std::endl;
    std::cout << std::flush;

    mjui_add(&ui0, defFile);

    std::cout << "9. SUCCESS!" << std::endl;

    // install GLFW mouse and keyboard callbacks
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
      // advance interactive simulation for 1/60 sec
      //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
      //  this loop will finish on time for the next frame to be rendered at 60 fps.
      //  Otherwise add a cpu timer and exit this loop when it is time to render.
      mjtNum simstart    = d->time;
      mjuiDef defJoint[] = {{mjITEM_SECTION, "Joint", mjPRESERVE, nullptr, "AJ"}, {mjITEM_END}};
      while (d->time - simstart < 1.0 / 60.0)
      {
        mj_step(m, d);
      }

      // render(window);
      //  get framebuffer viewport
      mjrRect viewport = {0, 0, 0, 0};
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

      // update scene and render
      mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
      mjr_render(viewport, &scn, &con);

      // swap OpenGL buffers (blocking call due to v-sync)
      glfwSwapBuffers(window);

      // process pending GUI events, call GLFW callbacks
      glfwPollEvents();
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
