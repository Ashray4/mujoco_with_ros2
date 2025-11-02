#include "mujoco_with_ros2/mujoco_render.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <ratio>
#include <string>
#include <type_traits>
#include <utility>

#include <mujoco/mjdata.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>


namespace mujoco_with_ros2 {

// section ids
enum
{
  // left ui
  SECT_OPTION = 0,
  // right ui
};

// Timer functions

mjtNum Timer()
{
  static auto start = mujoco_with_ros2::MujocoRender::Clock::now();
  auto elapsed      = Milliseconds(mujoco_with_ros2::MujocoRender::Clock::now() - start);
  return elapsed.count();
}

// Utility functions

const double zoom_increment = 0.02; // ratio of one click-wheel zoom increment to vertical extent

int ComputeFontScale(const mujoco_with_ros2::MujocoWithRos2UIAdapter& mujoco_with_ros2_ui)
{
  // compute framebuffer-to-window ratio
  auto [buf_width, buf_height] = mujoco_with_ros2_ui.GetFramebufferSize();
  auto [win_width, win_height] = mujoco_with_ros2_ui.GetWindowSize();
  double b2w                   = static_cast<double>(buf_width) / win_width;

  // compute PPI
  double PPI = b2w * mujoco_with_ros2_ui.GetDisplayPixelsPerInch();

  // estimate font scaling, guard against unrealistic PPI
  int fs;
  if (buf_width > win_width)
  {
    fs = mju_round(b2w * 100);
  }
  else if (PPI > 50 && PPI < 350)
  {
    fs = mju_round(PPI);
  }
  else
  {
    fs = 150;
  }
  fs = mju_round(fs * 0.02) * 50;
  fs = mjMIN(300, mjMAX(100, fs));

  return fs;
}

// UI Callbacks and Events
void UiLayout(mjuiState* state)
{
  mujoco_with_ros2::MujocoRender* sim =
    static_cast<mujoco_with_ros2::MujocoRender*>(state->userdata);
  mjrRect* rect = state->rect;

  // set number of rectangles
  state->nrect = 4;

  // rect 1: UI 0
  rect[1].left   = 0;
  rect[1].width  = sim->ui0_enable ? sim->ui0.width : 0;
  rect[1].bottom = 0;
  rect[1].height = rect[0].height;

  // rect 2: UI 1
  rect[2].width  = sim->ui1_enable ? sim->ui1.width : 0;
  rect[2].left   = mjMAX(0, rect[0].width - rect[2].width);
  rect[2].bottom = 0;
  rect[2].height = rect[0].height;

  // rect 3: 3D plot (everything else is an overlay)
  rect[3].left   = rect[1].width;
  rect[3].width  = mjMAX(0, rect[0].width - rect[1].width - rect[2].width);
  rect[3].bottom = 0;
  rect[3].height = rect[0].height;
}

// modify UI
void UiModify(mjUI* ui, mjuiState* state, mjrContext* con)
{
  mjui_resize(ui, con);

  // remake aux buffer only if missing or different
  int id = ui->auxid;
  if (con->auxFBO[id] == 0 || con->auxFBO_r[id] == 0 || con->auxColor[id] == 0 ||
      con->auxColor_r[id] == 0 || con->auxWidth[id] != ui->width ||
      con->auxHeight[id] != ui->maxheight || con->auxSamples[id] != ui->spacing.samples)
  {
    mjr_addAux(id, ui->width, ui->maxheight, ui->spacing.samples, con);
  }

  UiLayout(state);
  mjui_update(-1, -1, ui, state, con);
}

void UiEvent(mjuiState* state)
{
  mujoco_with_ros2::MujocoRender* sim =
    static_cast<mujoco_with_ros2::MujocoRender*>(state->userdata);

  // call UI 0 if event is directed to it
  if ((state->dragrect == sim->ui0.rectid) ||
      (state->dragrect == 0 && state->mouserect == sim->ui0.rectid) || state->type == mjEVENT_KEY)
  {
    // process UI event
    mjuiItem* it = mjui_event(&sim->ui0, state, &sim->mujoco_with_ros2_ui->mjr_context());

    // option section
    if (it && it->sectionid == SECT_OPTION)
    {
      if (it->pdata == &sim->spacing)
      {
        sim->ui0.spacing = mjui_themeSpacing(sim->spacing);
        sim->ui1.spacing = mjui_themeSpacing(sim->spacing);
      }
      else if (it->pdata == &sim->color)
      {
        sim->ui0.color = mjui_themeColor(sim->color);
        sim->ui1.color = mjui_themeColor(sim->color);
      }
      else if (it->pdata == &sim->font)
      {
        mjr_changeFont(50 * (sim->font + 1), &sim->mujoco_with_ros2_ui->mjr_context());
      }
      else if (it->pdata == &sim->fullscreen)
      {
        sim->mujoco_with_ros2_ui->ToggleFullscreen();
      }
      else if (it->pdata == &sim->vsync)
      {
        sim->mujoco_with_ros2_ui->SetVSync(sim->vsync);
      }

      // modify UI
      UiModify(&sim->ui0, state, &sim->mujoco_with_ros2_ui->mjr_context());
      UiModify(&sim->ui1, state, &sim->mujoco_with_ros2_ui->mjr_context());
    }

    // stop if UI processed event
    if (it != nullptr || (state->type == mjEVENT_KEY && state->key == 0))
    {
      return;
    }
  }

  //   // call UI 1 if event is directed to it
  //   if ((state->dragrect==sim->ui1.rectid) ||
  //       (state->dragrect==0 && state->mouserect==sim->ui1.rectid) ||
  //       state->type==mjEVENT_KEY) {
  //     // process UI event
  //     mjuiItem* it = mjui_event(&sim->ui1, state, &sim->mujoco_with_ros2_ui->mjr_context());

  //     // control section
  //     if (it && it->sectionid==SECT_CONTROL) {
  //       // clear controls
  //       if (it->itemid==0) {
  //         sim->pending_.zero_ctrl = true;
  //       }
  //     }

  //     // stop if UI processed event
  //     if (it!=nullptr || (state->type==mjEVENT_KEY && state->key==0)) {
  //       return;
  //     }
  //   }

  //   // shortcut not handled by UI
  //   if (state->type==mjEVENT_KEY && state->key!=0) {
  //     switch (state->key) {
  //     case ' ':                   // Mode
  //       if (!sim->is_passive_ && sim->m_) {
  //         sim->run = 1 - sim->run;
  //         sim->pert.active = 0;

  //         if (sim->run) sim->scrub_index = 0;  // reset scrubber

  //         mjui0_update_section(sim, -1);
  //       }
  //       break;

  //     case mjKEY_RIGHT:           // step forward
  //       if (!sim->is_passive_ && sim->m_ && !sim->run) {
  //         ClearTimers(sim->d_);

  //         // currently in scrubber: increment scrub, load state, update slider UI
  //         if (sim->scrub_index < 0) {
  //           sim->scrub_index++;
  //           sim->pending_.load_from_history = true;
  //           mjui0_update_section(sim, SECT_SIMULATION);
  //         }

  //         // not in scrubber: step, add to history buffer
  //         else {
  //           mj_step(sim->m_, sim->d_);
  //           sim->AddToHistory();
  //         }

  //         UpdateProfiler(sim, sim->m_, sim->d_);
  //         UpdateSensor(sim, sim->m_, sim->d_);
  //         UpdateSettings(sim, sim->m_);
  //       }
  //       break;

  //     case mjKEY_LEFT:           // step backward
  //       if (!sim->is_passive_ && sim->m_) {
  //         sim->run = 0;
  //         ClearTimers(sim->d_);

  //         // decrement scrub, load state
  //         sim->scrub_index = mjMAX(sim->scrub_index - 1, 1 - sim->nhistory_);
  //         sim->pending_.load_from_history = true;

  //         // update slider UI, profiler, sensor
  //         mjui0_update_section(sim, SECT_SIMULATION);
  //         UpdateProfiler(sim, sim->m_, sim->d_);
  //         UpdateSensor(sim, sim->m_, sim->d_);
  //       }
  //       break;

  //     case mjKEY_PAGE_UP:         // select parent body
  //       if ((sim->m_ || sim->is_passive_) && sim->pert.select > 0) {
  //         sim->pert.select = sim->body_parentid_[sim->pert.select];
  //         sim->pert.flexselect = -1;
  //         sim->pert.skinselect = -1;

  //         // stop perturbation if world reached
  //         if (sim->pert.select<=0) {
  //           sim->pert.active = 0;
  //         }
  //       }

  //       break;

  //     case ']':                   // cycle up fixed cameras
  //       if ((sim->m_ || !sim->is_passive_) && sim->ncam_) {
  //         sim->cam.type = mjCAMERA_FIXED;
  //         // camera = {0 or 1} are reserved for the free and tracking cameras
  //         if (sim->camera < 2 || sim->camera == 2 + sim->ncam_ - 1) {
  //           sim->camera = 2;
  //         } else {
  //           sim->camera += 1;
  //         }
  //         sim->cam.fixedcamid = sim->camera - 2;
  //         mjui0_update_section(sim, SECT_RENDERING);
  //       }
  //       break;

  //     case '[':                   // cycle down fixed cameras
  //       if ((sim->m_ || sim->is_passive_) && sim->ncam_) {
  //         sim->cam.type = mjCAMERA_FIXED;
  //         // camera = {0 or 1} are reserved for the free and tracking cameras
  //         if (sim->camera <= 2) {
  //           sim->camera = 2 + sim->ncam_-1;
  //         } else {
  //           sim->camera -= 1;
  //         }
  //         sim->cam.fixedcamid = sim->camera - 2;
  //         mjui0_update_section(sim, SECT_RENDERING);
  //       }
  //       break;

  //     case mjKEY_F6:                   // cycle frame visualisation
  //       if (sim->m_ || sim->is_passive_) {
  //         sim->opt.frame = (sim->opt.frame + 1) % mjNFRAME;
  //         mjui0_update_section(sim, SECT_RENDERING);
  //       }
  //       break;

  //     case mjKEY_F7:                   // cycle label visualisation
  //       if (sim->m_ || sim->is_passive_) {
  //         sim->opt.label = (sim->opt.label + 1) % mjNLABEL;
  //         mjui0_update_section(sim, SECT_RENDERING);
  //       }
  //       break;

  //     case mjKEY_ESCAPE:          // free camera
  //       sim->cam.type = mjCAMERA_FREE;
  //       sim->camera = 0;
  //       mjui0_update_section(sim, SECT_RENDERING);
  //       break;

  //     case '-':                   // slow down
  //       if (!sim->is_passive_) {
  //         int numclicks = sizeof(sim->percentRealTime) / sizeof(sim->percentRealTime[0]);
  //         if (sim->real_time_index < numclicks-1 && !state->shift) {
  //           sim->real_time_index++;
  //           sim->speed_changed = true;
  //         }
  //       }
  //       break;

  //     case '=':                   // speed up
  //       if (!sim->is_passive_ && sim->real_time_index > 0 && !state->shift) {
  //         sim->real_time_index--;
  //         sim->speed_changed = true;
  //       }
  //       break;

  //     case mjKEY_TAB:             // toggle left/right UI
  //       if (!state->shift) {
  //         // toggle left UI
  //         sim->ui0_enable = !sim->ui0_enable;
  //         UiModify(&sim->ui0, state, &sim->mujoco_with_ros2_ui->mjr_context());
  //       } else {
  //         // toggle right UI
  //         sim->ui1_enable = !sim->ui1_enable;
  //         UiModify(&sim->ui1, state, &sim->mujoco_with_ros2_ui->mjr_context());
  //       }
  //       break;
  //     }

  //     return;
  //   }

  // local pointers used below
  mjModel* model = sim->m_;
  mjData* data   = sim->d_;

  // 3D scroll
  if (state->type == mjEVENT_SCROLL && state->mouserect == 3 && model)
  {
    // emulate vertical mouse motion = 2% of window height
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -zoom_increment * state->sy, &sim->scn, &sim->cam);
    return;
  }

  // 3D press
  if (state->type == mjEVENT_PRESS && state->mouserect == 3)
  {
    // set perturbation
    int newperturb = 0;
    if (state->control && sim->pert.select > 0 && (sim->m_))
    {
      // right: translate;  left: rotate
      if (state->right)
      {
        newperturb = mjPERT_TRANSLATE;
      }
      else if (state->left)
      {
        newperturb = mjPERT_ROTATE;
      }
      //   if (newperturb && !sim->pert.active) {
      //     sim->pending_.newperturb = newperturb;
      //   }
    }

    // handle double-click
    if (state->doubleclick && (sim->m_))
    {
      //   sim->pending_.select = true;
      //   std::memcpy(&sim->pending_.select_state, state, sizeof(sim->pending_.select_state));

      // stop perturbation on select
      sim->pert.active = 0;
      //   sim->pending_.newperturb = 0;
    }

    return;
  }

  // 3D release
  if (state->type == mjEVENT_RELEASE && state->dragrect == 3 && (sim->m_))
  {
    // stop perturbation
    sim->pert.active = 0;
    // sim->pending_.newperturb = 0;
    return;
  }

  // 3D move
  if (state->type == mjEVENT_MOVE && state->dragrect == 3 && (sim->m_))
  {
    // determine action based on mouse button
    mjtMouse action;
    if (state->right)
    {
      action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (state->left)
    {
      action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
      action = mjMOUSE_ZOOM;
    }

    // move perturb or camera
    mjrRect r = state->rect[3];
    if (sim->pert.active)
    {
      mjv_movePerturb(
        model, data, action, state->dx / r.height, -state->dy / r.height, &sim->scn, &sim->pert);
    }
    else
    {
      mjv_moveCamera(
        model, action, state->dx / r.height, -state->dy / r.height, &sim->scn, &sim->cam);
    }
    return;
  }

  // Dropped files
  //   if (state->type == mjEVENT_FILESDROP && state->dropcount > 0 ) {
  //     while (sim->droploadrequest.load()) {}
  //     mju::strcpy_arr(sim->dropfilename, state->droppaths[0]);
  //     sim->droploadrequest.store(true);
  //     return;
  //   }

  // Redraw
  if (state->type == mjEVENT_REDRAW)
  {
    sim->Render();
    return;
  }
}

// loading and installing the current model
void MujocoRender::LoadOnRenderThread()
{
  this->m_ = this->mnew_;
  this->d_ = this->dnew_;

  ncam_ = this->m_->ncam;
  nkey_ = this->m_->nkey;
  body_parentid_.resize(this->m_->nbody);
  std::memcpy(body_parentid_.data(),
              this->m_->body_parentid,
              sizeof(this->m_->body_parentid[0]) * this->m_->nbody);

  jnt_type_.resize(this->m_->njnt);
  std::memcpy(jnt_type_.data(), this->m_->jnt_type, sizeof(this->m_->jnt_type[0]) * this->m_->njnt);

  jnt_group_.resize(this->m_->njnt);
  std::memcpy(
    jnt_group_.data(), this->m_->jnt_group, sizeof(this->m_->jnt_group[0]) * this->m_->njnt);

  jnt_qposadr_.resize(this->m_->njnt);
  std::memcpy(
    jnt_qposadr_.data(), this->m_->jnt_qposadr, sizeof(this->m_->jnt_qposadr[0]) * this->m_->njnt);

  //   jnt_range_.clear();
  //   jnt_range_.reserve(this->m_->njnt);
  //   for (int i = 0; i < this->m_->njnt; ++i) {
  //     if (this->m_->jnt_limited[i]) {
  //       jnt_range_.push_back(
  //           std::make_pair(this->m_->jnt_range[2 * i], this->m_->jnt_range[2 * i + 1]));
  //     } else {
  //       jnt_range_.push_back(std::nullopt);
  //     }
  //   }

  jnt_names_.clear();
  jnt_names_.reserve(this->m_->njnt);
  for (int i = 0; i < this->m_->njnt; ++i)
  {
    jnt_names_.emplace_back(this->m_->names + this->m_->name_jntadr[i]);
  }

  actuator_group_.resize(this->m_->nu);
  std::memcpy(actuator_group_.data(),
              this->m_->actuator_group,
              sizeof(this->m_->actuator_group[0]) * this->m_->nu);

  //   actuator_ctrlrange_.clear();
  //   actuator_ctrlrange_.reserve(this->m_->nu);
  //   for (int i = 0; i < this->m_->nu; ++i) {
  //     if (this->m_->actuator_ctrllimited[i]) {
  //       actuator_ctrlrange_.push_back(std::make_pair(
  //           this->m_->actuator_ctrlrange[2 * i], this->m_->actuator_ctrlrange[2 * i + 1]));
  //     } else {
  //       actuator_ctrlrange_.push_back(std::nullopt);
  //     }
  //   }

  actuator_names_.clear();
  actuator_names_.reserve(this->m_->nu);
  for (int i = 0; i < this->m_->nu; ++i)
  {
    actuator_names_.emplace_back(this->m_->names + this->m_->name_actuatoradr[i]);
  }

  //   equality_names_.clear();
  //   equality_names_.reserve(this->m_->neq);
  //   for (int i = 0; i < this->m_->neq; ++i) {
  //     equality_names_.emplace_back(this->m_->names + this->m_->name_eqadr[i]);
  //   }

  qpos_.resize(this->m_->nq);
  std::memcpy(qpos_.data(), this->d_->qpos, sizeof(this->d_->qpos[0]) * this->m_->nq);
  qpos_prev_ = qpos_;

  ctrl_.resize(this->m_->nu);
  std::memcpy(ctrl_.data(), this->d_->ctrl, sizeof(this->d_->ctrl[0]) * this->m_->nu);
  ctrl_prev_ = ctrl_;

  //   eq_active_.resize(this->m_->neq);
  //   std::memcpy(eq_active_.data(), this->d_->eq_active, sizeof(this->d_->eq_active[0]) *
  //   this->m_->neq); eq_active_prev_ = eq_active_;

  // allocate history buffer: smaller of {2000 states, 100 MB}

  constexpr int kMaxHistoryBytes = 1e8;

  // get state size, size of history buffer
  state_size_        = mj_stateSize(this->m_, mjSTATE_INTEGRATION);
  int state_bytes    = state_size_ * sizeof(mjtNum);
  int history_length = mjMIN(INT_MAX / state_bytes, 2000);
  int history_bytes  = mjMIN(state_bytes * history_length, kMaxHistoryBytes);
  nhistory_          = history_bytes / state_bytes;
  // allocate history buffer, reset cursor and UI slider
  history_.clear();
  history_.resize(nhistory_ * state_size_);
  history_cursor_ = 0;
  scrub_index     = 0;

  // fill buffer with initial state
  mj_getState(this->m_, this->d_, history_.data(), mjSTATE_INTEGRATION);
  for (int i = 1; i < nhistory_; ++i)
  {
    mju_copy(&history_[i * state_size_], history_.data(), state_size_);
  }


  // re-create scene
  mjv_makeScene(this->m_, &this->scn, kMaxGeom);

  this->mujoco_with_ros2_ui->RefreshMjrContext(this->m_, 50 * (this->font + 1));
  UiModify(&this->ui0, &this->uistate, &this->mujoco_with_ros2_ui->mjr_context());
  UiModify(&this->ui1, &this->uistate, &this->mujoco_with_ros2_ui->mjr_context());

  if (!this->mujoco_with_ros2_ui->IsGPUAccelerated())
  {
    this->scn.flags[mjRND_SHADOW]     = 0;
    this->scn.flags[mjRND_REFLECTION] = 0;
  }

  // THink about this later
//   if (this->user_scn)
//   {
//     Copy(this->user_scn->flags, this->scn.flags);
//     Copy(this->user_scn_flags_prev_, this->scn.flags);
//   }

  // clear perturbation state
  this->pert.active     = 0;
  this->pert.select     = 0;
  this->pert.flexselect = -1;
  this->pert.skinselect = -1;

  // align and scale view unless reloading the same file
//   if (this->filename[0] && mju::strcmp_arr(this->filename, this->previous_filename))
//   {
//     AlignAndScaleView(this, this->m_);
//     mju::strcpy_arr(this->previous_filename, this->filename);
//   }

  // update scene in managed mode
    mjv_updateScene(this->m_, this->d_, &this->opt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);


  // set window title to model name (Consider Later)
//   if (this->m_->names)
//   {
//     char title[200] = "MuJoCo : ";
//     mju::strcat_arr(title, this->m_->names);
//     mujoco_with_ros2_ui->SetWindowTitle(title);
//   }

  // rebuild UI sections
//   MakeUiSections(this, this->m_, this->d_);

  // full ui update
//   UiModify(&this->ui0, &this->uistate, &this->mujoco_with_ros2_ui->mjr_context());
//   UiModify(&this->ui1, &this->uistate, &this->mujoco_with_ros2_ui->mjr_context());
//   UpdateSettings(this, this->m_);

  // clear request
  this->loadrequest = 0;
  cond_loadrequest.notify_all();

  // set real time index
//   int numclicks   = sizeof(this->percentRealTime) / sizeof(this->percentRealTime[0]);
//   float min_error = 1e6;
//   float desired   = mju_log(100 * this->m_->vis.global.realtime);
//   for (int click = 0; click < numclicks; click++)
//   {
//     float error = mju_abs(mju_log(this->percentRealTime[click]) - desired);
//     if (error < min_error)
//     {
//       min_error             = error;
//       this->real_time_index = click;
//     }
//   }

  this->mnew_ = nullptr;
  this->dnew_ = nullptr;
}


// main render loop
void MujocoRender::RenderLoop()
{
  // Set timer callback (milliseconds)
  mjcb_time = Timer;

  // init abstract visualization
  mjv_defaultCamera(&this->cam);
  mjv_defaultOption(&this->opt);

  mjv_defaultScene(&this->scn);
  mjv_makeScene(nullptr, &this->scn, kMaxGeom);

  // select default font
  int fontscale = ComputeFontScale(*this->mujoco_with_ros2_ui);
  this->font    = fontscale / 50 - 1;

  // make empty context
  this->mujoco_with_ros2_ui->RefreshMjrContext(nullptr, fontscale);

  // init state and uis
  std::memset(&this->uistate, 0, sizeof(mjuiState));
  std::memset(&this->ui0, 0, sizeof(mjUI));
  std::memset(&this->ui1, 0, sizeof(mjUI));

  // Setup  UI and callbackd
  auto [buf_width, buf_height] = this->mujoco_with_ros2_ui->GetFramebufferSize();
  this->uistate.nrect          = 1;
  this->uistate.rect[0].width  = buf_width;
  this->uistate.rect[0].height = buf_height;

  this->ui0.spacing = mjui_themeSpacing(this->spacing);
  this->ui0.color   = mjui_themeColor(this->color);
  this->ui0.auxid   = 0;

  this->ui1.spacing = mjui_themeSpacing(this->spacing);
  this->ui1.color   = mjui_themeColor(this->color);
  this->ui1.rectid  = 2;
  this->ui1.auxid   = 1;

  // set GUI adapter callbacks
  this->uistate.userdata = this;
  this->mujoco_with_ros2_ui->SetEventCallback(UiEvent);
  this->mujoco_with_ros2_ui->SetLayoutCallback(UiLayout);

  // populate uis with standard sections, open some sections initially
  this->ui0.userdata = this;
  this->ui1.userdata = this;
  mjui_add(&this->ui0, this->def_option);
  this->ui0.sect[0].state = 1;
  this->ui0.sect[1].state = 1;
  this->ui0.sect[2].state = 1;
  UiModify(&this->ui0, &this->uistate, &this->mujoco_with_ros2_ui->mjr_context());
  UiModify(&this->ui1, &this->uistate, &this->mujoco_with_ros2_ui->mjr_context());

  // set VSync to initial value
  this->mujoco_with_ros2_ui->SetVSync(this->vsync);

  frames_          = 0;
  last_fps_update_ = mujoco_with_ros2::MujocoRender::Clock::now();

  while (!this->mujoco_with_ros2_ui->ShouldCloseWindow())
  {
    const MutexLock lock(this->mtx);

    // load model (not on first pass, to show "loading" label)
    if (this->loadrequest == 1)
    {
      this->LoadOnRenderThread();
    }
    else if (this->loadrequest == 2)
    {
      this->loadrequest = 1;
    }
  }
}

MujocoRender::MujocoRender(std::unique_ptr<MujocoWithRos2UIAdapter> mujoco_with_ros2_ui,
                           mjvCamera* cam,
                           mjvOption* opt,
                           mjvPerturb* pert)
  : cam(*cam)
  , opt(*opt)
  , pert(*pert)
  , mujoco_with_ros2_ui(std::move(mujoco_with_ros2_ui))
  , uistate(this->mujoco_with_ros2_ui->state())
{
  mjv_defaultScene(&scn);
}

} // namespace mujoco_with_ros2
