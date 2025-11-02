// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mujoco_with_ros2/mujoco_with_ros2_render.h"

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

#include "mujoco_with_ros2/array_safety.h"
#include "mujoco_with_ros2/mujoco_with_ros2_ui_adapter.h"
#include <mujoco/mjdata.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>

// When launched via an App Bundle on macOS, the working directory is the path to the App Bundle's
// resource directory. This causes files to be saved into the bundle, which is not the desired
// behavior. Instead, we open a save dialog box to ask the user where to put the file.
// Since the dialog box logic needs to be written in Objective-C, we separate it into a different
// source file.

static std::string GetSavePath(const char* filename)
{
  return filename;
}


namespace mujoco_with_ros2 {
namespace mju = ::mujoco_with_ros2::sample_util;

using Seconds      = std::chrono::duration<double>;
using Milliseconds = std::chrono::duration<double, std::milli>;

template <typename T>
inline bool IsDifferent(const T& a, const T& b)
{
  if constexpr (std::is_array_v<T>)
  {
    static_assert(std::rank_v<T> == 1);
    for (int i = 0; i < std::extent_v<T>; ++i)
    {
      if (a[i] != b[i])
      {
        return true;
      }
    }
    return false;
  }
  else
  {
    return a != b;
  }
}

template <typename T>
inline void CopyScalar(T& dst, const T& src)
{
  dst = src;
}

template <typename T, int N>
inline void CopyArray(T (&dst)[N], const T (&src)[N])
{
  for (int i = 0; i < N; ++i)
  {
    dst[i] = src[i];
  }
}

template <typename T>
inline void Copy(T& dst, const T& src)
{
  if constexpr (std::is_array_v<T>)
  {
    CopyArray(dst, src);
  }
  else
  {
    CopyScalar(dst, src);
  }
}

//------------------------------------------- global -----------------------------------------------

const double zoom_increment = 0.02; // ratio of one click-wheel zoom increment to vertical extent

// section ids
enum
{
  // left ui
  SECT_FILE = 0,
  SECT_OPTION,
  SECT_SIMULATION,
  SECT_WATCH,
  SECT_PHYSICS,
  SECT_RENDERING,
  SECT_VISUALIZATION,
  SECT_GROUP,
  NSECT0,

  // right ui
  SECT_JOINT = 0,
  SECT_CONTROL,
  SECT_EQUALITY,
  NSECT1
};

// file section of UI
const mjuiDef defFile[] = {{mjITEM_SECTION, "File", mjPRESERVE, nullptr, "AF"},
                           {mjITEM_BUTTON, "Save xml", 2, nullptr, ""},
                           {mjITEM_BUTTON, "Save mjb", 2, nullptr, ""},
                           {mjITEM_BUTTON, "Print model", 2, nullptr, "CM"},
                           {mjITEM_BUTTON, "Print data", 2, nullptr, "CD"},
                           {mjITEM_BUTTON, "Quit", 1, nullptr, "CQ"},
                           {mjITEM_BUTTON, "Screenshot", 2, nullptr, "CP"},
                           {mjITEM_END}};

// help strings
const char help_content[] = "Space\n"
                            "+  -\n"
                            "Left / Right arrow\n"
                            "Tab / Shift-Tab\n"
                            "[  ]\n"
                            "Esc\n"
                            "Double-click\n"
                            "Page Up\n"
                            "Right double-click\n"
                            "Ctrl Right double-click\n"
                            "Scroll, middle drag\n"
                            "Left drag\n"
                            "[Shift] right drag\n"
                            "Ctrl [Shift] drag\n"
                            "Ctrl [Shift] right drag\n"
                            "F1\n"
                            "F2\n"
                            "F3\n"
                            "F4\n"
                            "F5\n"
                            "UI right-button hold\n"
                            "UI title double-click";

const char help_title[] = "Play / Pause\n"
                          "Speed Up / Down\n"
                          "Step Back / Forward\n"
                          "Toggle Left / Right UI\n"
                          "Cycle cameras\n"
                          "Free camera\n"
                          "Select\n"
                          "Select parent\n"
                          "Center camera\n"
                          "Tracking camera\n"
                          "Zoom\n"
                          "View Orbit\n"
                          "View Pan\n"
                          "Object Rotate\n"
                          "Object Translate\n"
                          "Help\n"
                          "Info\n"
                          "Profiler\n"
                          "Sensors\n"
                          "Full screen\n"
                          "Show UI shortcuts\n"
                          "Expand/collapse all";


//-------------------------------- profiler, sensor, info, watch -----------------------------------

// number of lines in the Constraint ("Counts") and Cost ("Convergence") figures
static constexpr int kConstraintNum = 5;
static constexpr int kCostNum       = 3;

// update an entire section of ui0
static void mjui0_update_section(mujoco_with_ros2::MujocoWithROS2Render* sim, int section)
{
  mjui_update(section, -1, &sim->ui0, &sim->uistate, &sim->platform_ui->mjr_context());
}

// prepare info text
void UpdateInfoText(mujoco_with_ros2::MujocoWithROS2Render* sim,
                    const mjModel* m,
                    const mjData* d,
                    char (&title)[mujoco_with_ros2::MujocoWithROS2Render::kMaxFilenameLength],
                    char (&content)[mujoco_with_ros2::MujocoWithROS2Render::kMaxFilenameLength])
{
  char tmp[20];

  // number of islands with statistics
  int nisland = mjMAX(1, mjMIN(d->nisland, mjNISLAND));

  // compute solver error (maximum over islands)
  mjtNum solerr = 0;
  for (int i = 0; i < nisland; i++)
  {
    mjtNum solerr_i = 0;
    if (d->solver_niter[i])
    {
      int ind                  = mjMIN(d->solver_niter[i], mjNSOLVER) - 1;
      const mjSolverStat* stat = d->solver + i * mjNSOLVER + ind;
      solerr_i                 = mju_min(stat->improvement, stat->gradient);
      if (solerr_i == 0)
      {
        solerr_i = mju_max(stat->improvement, stat->gradient);
      }
    }
    solerr = mju_max(solerr, solerr_i);
  }
  solerr = mju_log10(mju_max(mjMINVAL, solerr));

  // format FPS text
  char fps[10];
  if (sim->fps_ < 1)
  {
    mju::sprintf_arr(fps, "%0.1f ", sim->fps_);
  }
  else
  {
    mju::sprintf_arr(fps, "%.0f ", sim->fps_);
  }

  // total iterations of all islands with statistics
  int solver_niter = 0;
  for (int i = 0; i < nisland; i++)
  {
    solver_niter += d->solver_niter[i];
  }

  // prepare info text
  mju::strcpy_arr(title, "Time\nSize\nCPU\nSolver   \nFPS\nMemory");
  mju::sprintf_arr(
    content,
    "%-9.3f\n%d  (%d con)\n%.3f\n%.1f  (%d it)\n%s\n%.1f%% of %s",
    d->time,
    d->nefc,
    d->ncon,
    sim->run ? d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number)
             : d->timer[mjTIMER_FORWARD].duration / mjMAX(1, d->timer[mjTIMER_FORWARD].number),
    solerr,
    solver_niter,
    fps,
    100 * d->maxuse_arena / (double)(d->narena),
    mju_writeNumBytes(d->narena));

  // add Energy if enabled
  {
    if (mjENABLED(mjENBL_ENERGY))
    {
      mju::sprintf_arr(tmp, "\n%.3f", d->energy[0] + d->energy[1]);
      mju::strcat_arr(content, tmp);
      mju::strcat_arr(title, "\nEnergy");
    }

    // add FwdInv if enabled
    if (mjENABLED(mjENBL_FWDINV))
    {
      mju::sprintf_arr(tmp,
                       "\n%.1f %.1f",
                       mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[0])),
                       mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[1])));
      mju::strcat_arr(content, tmp);
      mju::strcat_arr(title, "\nFwdInv");
    }
  }
}

// sprintf forwarding, to avoid compiler warning in x-macro
void PrintField(char (&str)[mjMAXUINAME], void* ptr)
{
  mju::sprintf_arr(str, "%g", *static_cast<mjtNum*>(ptr));
}

// update watch
void UpdateWatch(mujoco_with_ros2::MujocoWithROS2Render* sim, const mjModel* m, const mjData* d)
{
  // clear
  sim->ui0.sect[SECT_WATCH].item[2].multi.nelem = 1;
  mju::strcpy_arr(sim->ui0.sect[SECT_WATCH].item[2].multi.name[0], "invalid field");

  // prepare symbols needed by xmacro
  MJDATA_POINTERS_PREAMBLE(m);

// find specified field in mjData arrays, update value
#define X(TYPE, NAME, NR, NC)                                                                      \
  if (!mju::strcmp_arr(#NAME, sim->field) && !mju::strcmp_arr(#TYPE, "mjtNum"))                    \
  {                                                                                                \
    if (sim->index >= 0 && sim->index < m->NR * NC)                                                \
    {                                                                                              \
      PrintField(sim->ui0.sect[SECT_WATCH].item[2].multi.name[0], d->NAME + sim->index);           \
    }                                                                                              \
    else                                                                                           \
    {                                                                                              \
      mju::strcpy_arr(sim->ui0.sect[SECT_WATCH].item[2].multi.name[0], "invalid index");           \
    }                                                                                              \
    return;                                                                                        \
  }

  MJDATA_POINTERS
#undef X
}


//---------------------------------- UI construction -----------------------------------------------

// make physics section of UI
void MakePhysicsSection(mujoco_with_ros2::MujocoWithROS2Render* sim)
{
  mjOption* opt        = &sim->m_->opt;
  mjuiDef defPhysics[] = {
    {mjITEM_SECTION, "Physics", mjPRESERVE, nullptr, "AP"},
    {mjITEM_SELECT, "Integrator", 2, &(opt->integrator), "Euler\nRK4\nimplicit\nimplicitfast"},
    {mjITEM_SELECT, "Cone", 2, &(opt->cone), "Pyramidal\nElliptic"},
    {mjITEM_SELECT, "Jacobian", 2, &(opt->jacobian), "Dense\nSparse\nAuto"},
    {mjITEM_SELECT, "Solver", 2, &(opt->solver), "PGS\nCG\nNewton"},
    {mjITEM_SEPARATOR, "Algorithmic Parameters", mjPRESERVE},
    {mjITEM_EDITNUM, "Timestep", 2, &(opt->timestep), "1 0 1"},
    {mjITEM_EDITINT, "Iterations", 2, &(opt->iterations), "1 0 1000"},
    {mjITEM_EDITNUM, "Tolerance", 2, &(opt->tolerance), "1 0 1"},
    {mjITEM_EDITINT, "LS Iter", 2, &(opt->ls_iterations), "1 0 100"},
    {mjITEM_EDITNUM, "LS Tol", 2, &(opt->ls_tolerance), "1 0 0.1"},
    {mjITEM_EDITINT, "Noslip Iter", 2, &(opt->noslip_iterations), "1 0 1000"},
    {mjITEM_EDITNUM, "Noslip Tol", 2, &(opt->noslip_tolerance), "1 0 1"},
    {mjITEM_EDITINT, "CCD Iter", 2, &(opt->ccd_iterations), "1 0 1000"},
    {mjITEM_EDITNUM, "CCD Tol", 2, &(opt->ccd_tolerance), "1 0 1"},
    {mjITEM_EDITINT, "SDF Iter", 2, &(opt->sdf_iterations), "1 1 20"},
    {mjITEM_EDITINT, "SDF Init", 2, &(opt->sdf_initpoints), "1 1 100"},
    {mjITEM_SEPARATOR, "Physical Parameters", mjPRESERVE},
    {mjITEM_EDITNUM, "Gravity", 2, opt->gravity, "3"},
    {mjITEM_EDITNUM, "Wind", 2, opt->wind, "3"},
    {mjITEM_EDITNUM, "Magnetic", 2, opt->magnetic, "3"},
    {mjITEM_EDITNUM, "Density", 2, &(opt->density), "1"},
    {mjITEM_EDITNUM, "Viscosity", 2, &(opt->viscosity), "1"},
    {mjITEM_EDITNUM, "Imp Ratio", 2, &(opt->impratio), "1"},
    {mjITEM_SEPARATOR, "Disable Flags", mjPRESERVE},
    {mjITEM_END}};
  mjuiDef defEnableFlags[]     = {{mjITEM_SEPARATOR, "Enable Flags", mjPRESERVE}, {mjITEM_END}};
  mjuiDef defOverride[]        = {{mjITEM_SEPARATOR, "Contact Override", mjPRESERVE},
                                  {mjITEM_EDITNUM, "Margin", 2, &(opt->o_margin), "1"},
                                  {mjITEM_EDITNUM, "Sol Imp", 2, &(opt->o_solimp), "5"},
                                  {mjITEM_EDITNUM, "Sol Ref", 2, &(opt->o_solref), "2"},
                                  {mjITEM_EDITNUM, "Friction", 2, &(opt->o_friction), "5"},
                                  {mjITEM_END}};
  mjuiDef defDisableActuator[] = {{mjITEM_SEPARATOR, "Actuator Group Enable", mjPRESERVE},
                                  {mjITEM_CHECKBYTE, "Act Group 0", 2, sim->enableactuator + 0, ""},
                                  {mjITEM_CHECKBYTE, "Act Group 1", 2, sim->enableactuator + 1, ""},
                                  {mjITEM_CHECKBYTE, "Act Group 2", 2, sim->enableactuator + 2, ""},
                                  {mjITEM_CHECKBYTE, "Act Group 3", 2, sim->enableactuator + 3, ""},
                                  {mjITEM_CHECKBYTE, "Act Group 4", 2, sim->enableactuator + 4, ""},
                                  {mjITEM_CHECKBYTE, "Act Group 5", 2, sim->enableactuator + 5, ""},
                                  {mjITEM_END}};

  // add physics
  mjui_add(&sim->ui0, defPhysics);

  // add flags programmatically
  mjuiDef defFlag[] = {{mjITEM_CHECKINT, "", 2, nullptr, ""}, {mjITEM_END}};
  for (int i = 0; i < mjNDISABLE; i++)
  {
    mju::strcpy_arr(defFlag[0].name, mjDISABLESTRING[i]);
    defFlag[0].pdata = sim->disable + i;
    mjui_add(&sim->ui0, defFlag);
  }
  mjui_add(&sim->ui0, defEnableFlags);
  for (int i = 0; i < mjNENABLE; i++)
  {
    mju::strcpy_arr(defFlag[0].name, mjENABLESTRING[i]);
    defFlag[0].pdata = sim->enable + i;
    mjui_add(&sim->ui0, defFlag);
  }
  // add contact override
  mjui_add(&sim->ui0, defOverride);

  // add actuator group enable/disable
  mjui_add(&sim->ui0, defDisableActuator);

  // make some subsections closed by default
  for (int i = 0; i < sim->ui0.sect[SECT_PHYSICS].nitem; i++)
  {
    mjuiItem* it = sim->ui0.sect[SECT_PHYSICS].item + i;

    // close less useful subsections
    if (it->type == mjITEM_SEPARATOR)
    {
      if (mju::strcmp_arr(it->name, "Actuator Group Enable") &&
          mju::strcmp_arr(it->name, "Contact Override") &&
          mju::strcmp_arr(it->name, "Physical Parameters"))
      {
        it->state = mjSEPCLOSED + 1;
      }
    }
  }
}


// make rendering section of UI
void MakeRenderingSection(mujoco_with_ros2::MujocoWithROS2Render* sim, const mjModel* m)
{
  mjuiDef defRendering[] = {
    {mjITEM_SECTION, "Rendering", mjPRESERVE, nullptr, "AR"},
    {mjITEM_SELECT, "Camera", 2, &(sim->camera), "Free\nTracking"},
    {mjITEM_SELECT,
     "Label",
     2,
     &(sim->opt.label),
     "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\n"
     "Actuator\nConstraint\nFlex\nSkin\nSelection\nSel Pnt\nContact\nForce\nIsland"},
    {mjITEM_SELECT,
     "Frame",
     2,
     &(sim->opt.frame),
     "None\nBody\nGeom\nSite\nCamera\nLight\nContact\nWorld"},
    {mjITEM_BUTTON, "Copy camera", 2, nullptr, ""},
    {mjITEM_SEPARATOR, "Model Elements", 1},
    {mjITEM_END}};
  mjuiDef defOpenGL[] = {{mjITEM_SEPARATOR, "OpenGL Effects", 1}, {mjITEM_END}};

  // add model cameras, up to UI limit
  for (int i = 0; i < mjMIN(m->ncam, mjMAXUIMULTI - 2); i++)
  {
    // prepare name
    char camname[mjMAXUINAME] = "\n";
    if (m->names[m->name_camadr[i]])
    {
      mju::strcat_arr(camname, m->names + m->name_camadr[i]);
    }
    else
    {
      mju::sprintf_arr(camname, "\nCamera %d", i);
    }

    // check string length
    if (mju::strlen_arr(camname) + mju::strlen_arr(defRendering[1].other) >= mjMAXUITEXT - 1)
    {
      break;
    }

    // add camera
    mju::strcat_arr(defRendering[1].other, camname);
  }

  // add rendering standard
  mjui_add(&sim->ui0, defRendering);

  // add flags programmatically
  mjuiDef defFlag[] = {{mjITEM_CHECKBYTE, "", 2, nullptr, ""}, {mjITEM_END}};
  for (int i = 0; i < mjNVISFLAG; i++)
  {
    // set name
    mju::strcpy_arr(defFlag[0].name, mjVISSTRING[i][0]);

    // set shortcut and data
    if (mjVISSTRING[i][2][0])
    {
      mju::sprintf_arr(defFlag[0].other, " %s", mjVISSTRING[i][2]);
    }
    else
    {
      mju::sprintf_arr(defFlag[0].other, "");
    }
    defFlag[0].pdata = sim->opt.flags + i;
    mjui_add(&sim->ui0, defFlag);
  }

  // create tree slider
  mjuiDef defTree[] = {{mjITEM_SLIDERINT, "Tree depth", 2, &sim->opt.bvh_depth, "0 20"},
                       {mjITEM_SLIDERINT, "Flex layer", 2, &sim->opt.flex_layer, "0 10"},
                       {mjITEM_END}};
  mjui_add(&sim->ui0, defTree);

  // add rendering flags
  mjui_add(&sim->ui0, defOpenGL);
  for (int i = 0; i < mjNRNDFLAG; i++)
  {
    // set name
    mju::strcpy_arr(defFlag[0].name, mjRNDSTRING[i][0]);

    // set shortcut and data
    if (mjRNDSTRING[i][2][0])
    {
      mju::sprintf_arr(defFlag[0].other, " %s", mjRNDSTRING[i][2]);
    }
    else
    {
      mju::sprintf_arr(defFlag[0].other, "");
    }
    defFlag[0].pdata = sim->scn.flags + i;
    mjui_add(&sim->ui0, defFlag);
  }
}

// make visualization section of UI
void MakeVisualizationSection(mujoco_with_ros2::MujocoWithROS2Render* sim, const mjModel* m)
{
  mjStatistic* stat = &sim->m_->stat;
  mjVisual* vis     = &sim->m_->vis;

  mjuiDef defVisualization[] = {
    {mjITEM_SECTION, "Visualization", mjPRESERVE, nullptr, "AV"},
    {mjITEM_SEPARATOR, "Headlight", 1},
    {mjITEM_RADIO, "Active", 2, &(vis->headlight.active), "Off\nOn"},
    {mjITEM_EDITFLOAT, "Ambient", 2, &(vis->headlight.ambient), "3"},
    {mjITEM_EDITFLOAT, "Diffuse", 2, &(vis->headlight.diffuse), "3"},
    {mjITEM_EDITFLOAT, "Specular", 2, &(vis->headlight.specular), "3"},
    {mjITEM_SEPARATOR, "Free Camera", 1},
    {mjITEM_RADIO, "Orthographic", 2, &(vis->global.orthographic), "No\nYes"},
    {mjITEM_EDITFLOAT, "Field of view", 2, &(vis->global.fovy), "1"},
    {mjITEM_EDITNUM, "Center", 2, &(stat->center), "3"},
    {mjITEM_EDITFLOAT, "Azimuth", 2, &(vis->global.azimuth), "1"},
    {mjITEM_EDITFLOAT, "Elevation", 2, &(vis->global.elevation), "1"},
    {mjITEM_BUTTON, "Align", 2, nullptr, "CA"},
    {mjITEM_SEPARATOR, "Global", 1},
    {mjITEM_EDITNUM, "Extent", 2, &(stat->extent), "1"},
    {mjITEM_RADIO, "Inertia", 2, &(vis->global.ellipsoidinertia), "Box\nEllipsoid"},
    {mjITEM_RADIO, "BVH active", 5, &(vis->global.bvactive), "False\nTrue"},
    {mjITEM_SEPARATOR, "Map", 1},
    {mjITEM_EDITFLOAT, "Stiffness", 2, &(vis->map.stiffness), "1"},
    {mjITEM_EDITFLOAT, "Rot stiffness", 2, &(vis->map.stiffnessrot), "1"},
    {mjITEM_EDITFLOAT, "Force", 2, &(vis->map.force), "1"},
    {mjITEM_EDITFLOAT, "Torque", 2, &(vis->map.torque), "1"},
    {mjITEM_EDITFLOAT, "Alpha", 2, &(vis->map.alpha), "1"},
    {mjITEM_EDITFLOAT, "Fog start", 2, &(vis->map.fogstart), "1"},
    {mjITEM_EDITFLOAT, "Fog end", 2, &(vis->map.fogend), "1"},
    {mjITEM_EDITFLOAT, "Z near", 2, &(vis->map.znear), "1"},
    {mjITEM_EDITFLOAT, "Z far", 2, &(vis->map.zfar), "1"},
    {mjITEM_EDITFLOAT, "Haze", 2, &(vis->map.haze), "1"},
    {mjITEM_EDITFLOAT, "Shadow clip", 2, &(vis->map.shadowclip), "1"},
    {mjITEM_EDITFLOAT, "Shadow scale", 2, &(vis->map.shadowscale), "1"},
    {mjITEM_SEPARATOR, "Scale", mjPRESERVE},
    {mjITEM_EDITNUM, "All (meansize)", 2, &(stat->meansize), "1"},
    {mjITEM_EDITFLOAT, "Force width", 2, &(vis->scale.forcewidth), "1"},
    {mjITEM_EDITFLOAT, "Contact width", 2, &(vis->scale.contactwidth), "1"},
    {mjITEM_EDITFLOAT, "Contact height", 2, &(vis->scale.contactheight), "1"},
    {mjITEM_EDITFLOAT, "Connect", 2, &(vis->scale.connect), "1"},
    {mjITEM_EDITFLOAT, "Com", 2, &(vis->scale.com), "1"},
    {mjITEM_EDITFLOAT, "Camera", 2, &(vis->scale.camera), "1"},
    {mjITEM_EDITFLOAT, "Light", 2, &(vis->scale.light), "1"},
    {mjITEM_EDITFLOAT, "Select point", 2, &(vis->scale.selectpoint), "1"},
    {mjITEM_EDITFLOAT, "Joint length", 2, &(vis->scale.jointlength), "1"},
    {mjITEM_EDITFLOAT, "Joint width", 2, &(vis->scale.jointwidth), "1"},
    {mjITEM_EDITFLOAT, "Actuator length", 2, &(vis->scale.actuatorlength), "1"},
    {mjITEM_EDITFLOAT, "Actuator width", 2, &(vis->scale.actuatorwidth), "1"},
    {mjITEM_EDITFLOAT, "Frame length", 2, &(vis->scale.framelength), "1"},
    {mjITEM_EDITFLOAT, "Frame width", 2, &(vis->scale.framewidth), "1"},
    {mjITEM_EDITFLOAT, "Constraint", 2, &(vis->scale.constraint), "1"},
    {mjITEM_EDITFLOAT, "Slider-crank", 2, &(vis->scale.slidercrank), "1"},
    {mjITEM_SEPARATOR, "RGBA", mjPRESERVE},
    {mjITEM_EDITFLOAT, "fog", 2, &(vis->rgba.fog), "4"},
    {mjITEM_EDITFLOAT, "haze", 2, &(vis->rgba.haze), "4"},
    {mjITEM_EDITFLOAT, "force", 2, &(vis->rgba.force), "4"},
    {mjITEM_EDITFLOAT, "inertia", 2, &(vis->rgba.inertia), "4"},
    {mjITEM_EDITFLOAT, "joint", 2, &(vis->rgba.joint), "4"},
    {mjITEM_EDITFLOAT, "actuator", 2, &(vis->rgba.actuator), "4"},
    {mjITEM_EDITFLOAT, "actnegative", 2, &(vis->rgba.actuatornegative), "4"},
    {mjITEM_EDITFLOAT, "actpositive", 2, &(vis->rgba.actuatorpositive), "4"},
    {mjITEM_EDITFLOAT, "com", 2, &(vis->rgba.com), "4"},
    {mjITEM_EDITFLOAT, "camera", 2, &(vis->rgba.camera), "4"},
    {mjITEM_EDITFLOAT, "light", 2, &(vis->rgba.light), "4"},
    {mjITEM_EDITFLOAT, "selectpoint", 2, &(vis->rgba.selectpoint), "4"},
    {mjITEM_EDITFLOAT, "connect", 2, &(vis->rgba.connect), "4"},
    {mjITEM_EDITFLOAT, "contactpoint", 2, &(vis->rgba.contactpoint), "4"},
    {mjITEM_EDITFLOAT, "contactforce", 2, &(vis->rgba.contactforce), "4"},
    {mjITEM_EDITFLOAT, "contactfriction", 2, &(vis->rgba.contactfriction), "4"},
    {mjITEM_EDITFLOAT, "contacttorque", 2, &(vis->rgba.contacttorque), "4"},
    {mjITEM_EDITFLOAT, "contactgap", 2, &(vis->rgba.contactgap), "4"},
    {mjITEM_EDITFLOAT, "rangefinder", 2, &(vis->rgba.rangefinder), "4"},
    {mjITEM_EDITFLOAT, "constraint", 2, &(vis->rgba.constraint), "4"},
    {mjITEM_EDITFLOAT, "slidercrank", 2, &(vis->rgba.slidercrank), "4"},
    {mjITEM_EDITFLOAT, "crankbroken", 2, &(vis->rgba.crankbroken), "4"},
    {mjITEM_EDITFLOAT, "frustum", 2, &(vis->rgba.frustum), "4"},
    {mjITEM_EDITFLOAT, "bv", 2, &(vis->rgba.bv), "4"},
    {mjITEM_EDITFLOAT, "bvactive", 2, &(vis->rgba.bvactive), "4"},
    {mjITEM_END}};

  // add visualization section
  mjui_add(&sim->ui0, defVisualization);
}

// make group section of UI
void MakeGroupSection(mujoco_with_ros2::MujocoWithROS2Render* sim)
{
  mjuiDef defGroup[] = {{mjITEM_SECTION, "Group enable", mjPRESERVE, nullptr, "AG"},
                        {mjITEM_SEPARATOR, "Geom groups", 1},
                        {mjITEM_CHECKBYTE, "Geom 0", 2, sim->opt.geomgroup, " 0"},
                        {mjITEM_CHECKBYTE, "Geom 1", 2, sim->opt.geomgroup + 1, " 1"},
                        {mjITEM_CHECKBYTE, "Geom 2", 2, sim->opt.geomgroup + 2, " 2"},
                        {mjITEM_CHECKBYTE, "Geom 3", 2, sim->opt.geomgroup + 3, " 3"},
                        {mjITEM_CHECKBYTE, "Geom 4", 2, sim->opt.geomgroup + 4, " 4"},
                        {mjITEM_CHECKBYTE, "Geom 5", 2, sim->opt.geomgroup + 5, " 5"},
                        {mjITEM_SEPARATOR, "Site groups", 1},
                        {mjITEM_CHECKBYTE, "Site 0", 2, sim->opt.sitegroup, "S0"},
                        {mjITEM_CHECKBYTE, "Site 1", 2, sim->opt.sitegroup + 1, "S1"},
                        {mjITEM_CHECKBYTE, "Site 2", 2, sim->opt.sitegroup + 2, "S2"},
                        {mjITEM_CHECKBYTE, "Site 3", 2, sim->opt.sitegroup + 3, "S3"},
                        {mjITEM_CHECKBYTE, "Site 4", 2, sim->opt.sitegroup + 4, "S4"},
                        {mjITEM_CHECKBYTE, "Site 5", 2, sim->opt.sitegroup + 5, "S5"},
                        {mjITEM_SEPARATOR, "Joint groups", 1},
                        {mjITEM_CHECKBYTE, "Joint 0", 2, sim->opt.jointgroup, ""},
                        {mjITEM_CHECKBYTE, "Joint 1", 2, sim->opt.jointgroup + 1, ""},
                        {mjITEM_CHECKBYTE, "Joint 2", 2, sim->opt.jointgroup + 2, ""},
                        {mjITEM_CHECKBYTE, "Joint 3", 2, sim->opt.jointgroup + 3, ""},
                        {mjITEM_CHECKBYTE, "Joint 4", 2, sim->opt.jointgroup + 4, ""},
                        {mjITEM_CHECKBYTE, "Joint 5", 2, sim->opt.jointgroup + 5, ""},
                        {mjITEM_SEPARATOR, "Tendon groups", 1},
                        {mjITEM_CHECKBYTE, "Tendon 0", 2, sim->opt.tendongroup, ""},
                        {mjITEM_CHECKBYTE, "Tendon 1", 2, sim->opt.tendongroup + 1, ""},
                        {mjITEM_CHECKBYTE, "Tendon 2", 2, sim->opt.tendongroup + 2, ""},
                        {mjITEM_CHECKBYTE, "Tendon 3", 2, sim->opt.tendongroup + 3, ""},
                        {mjITEM_CHECKBYTE, "Tendon 4", 2, sim->opt.tendongroup + 4, ""},
                        {mjITEM_CHECKBYTE, "Tendon 5", 2, sim->opt.tendongroup + 5, ""},
                        {mjITEM_SEPARATOR, "Actuator groups", 1},
                        {mjITEM_CHECKBYTE, "Actuator 0", 2, sim->opt.actuatorgroup, ""},
                        {mjITEM_CHECKBYTE, "Actuator 1", 2, sim->opt.actuatorgroup + 1, ""},
                        {mjITEM_CHECKBYTE, "Actuator 2", 2, sim->opt.actuatorgroup + 2, ""},
                        {mjITEM_CHECKBYTE, "Actuator 3", 2, sim->opt.actuatorgroup + 3, ""},
                        {mjITEM_CHECKBYTE, "Actuator 4", 2, sim->opt.actuatorgroup + 4, ""},
                        {mjITEM_CHECKBYTE, "Actuator 5", 2, sim->opt.actuatorgroup + 5, ""},
                        {mjITEM_SEPARATOR, "Flex groups", 1},
                        {mjITEM_CHECKBYTE, "Flex 0", 2, sim->opt.flexgroup, ""},
                        {mjITEM_CHECKBYTE, "Flex 1", 2, sim->opt.flexgroup + 1, ""},
                        {mjITEM_CHECKBYTE, "Flex 2", 2, sim->opt.flexgroup + 2, ""},
                        {mjITEM_CHECKBYTE, "Flex 3", 2, sim->opt.flexgroup + 3, ""},
                        {mjITEM_CHECKBYTE, "Flex 4", 2, sim->opt.flexgroup + 4, ""},
                        {mjITEM_CHECKBYTE, "Flex 5", 2, sim->opt.flexgroup + 5, ""},
                        {mjITEM_SEPARATOR, "Skin groups", 1},
                        {mjITEM_CHECKBYTE, "Skin 0", 2, sim->opt.skingroup, ""},
                        {mjITEM_CHECKBYTE, "Skin 1", 2, sim->opt.skingroup + 1, ""},
                        {mjITEM_CHECKBYTE, "Skin 2", 2, sim->opt.skingroup + 2, ""},
                        {mjITEM_CHECKBYTE, "Skin 3", 2, sim->opt.skingroup + 3, ""},
                        {mjITEM_CHECKBYTE, "Skin 4", 2, sim->opt.skingroup + 4, ""},
                        {mjITEM_CHECKBYTE, "Skin 5", 2, sim->opt.skingroup + 5, ""},
                        {mjITEM_END}};

  // add section
  mjui_add(&sim->ui0, defGroup);
}

// make joint section of UI
void MakeJointSection(mujoco_with_ros2::MujocoWithROS2Render* sim)
{
  mjuiDef defJoint[]  = {{mjITEM_SECTION, "Joint", mjPRESERVE, nullptr, "AJ"}, {mjITEM_END}};
  mjuiDef defSlider[] = {{mjITEM_SLIDERNUM, "", 2, nullptr, "0 1"}, {mjITEM_END}};

  // add section
  mjui_add(&sim->ui1, defJoint);
  defSlider[0].state = 4;

  // add scalar joints, exit if UI limit reached
  int itemcnt = 0;
  for (int i = 0; i < sim->jnt_type_.size() && itemcnt < mjMAXUIITEM; i++)
  {
    if ((sim->jnt_type_[i] == mjJNT_HINGE || sim->jnt_type_[i] == mjJNT_SLIDE))
    {
      // skip if joint group is disabled
      if (!sim->opt.jointgroup[mjMAX(0, mjMIN(mjNGROUP - 1, sim->jnt_group_[i]))])
      {
        continue;
      }
      defSlider[0].pdata = &sim->qpos_[sim->jnt_qposadr_[i]];

      if (!sim->jnt_names_[i].empty())
      {
        mju::strcpy_arr(defSlider[0].name, sim->jnt_names_[i].c_str());
      }
      else
      {
        mju::sprintf_arr(defSlider[0].name, "joint %d", i);
      }

      //   // set range
      //   if (sim->jnt_range_[i].has_value())
      //     mju::sprintf_arr(defSlider[0].other, "%.4g %.4g",
      //                      sim->jnt_range_[i]->first, sim->jnt_range_[i]->second);
      //   else if (sim->jnt_type_[i]==mjJNT_SLIDE) {
      //     mju::strcpy_arr(defSlider[0].other, "-1 1");
      //   } else {
      mju::strcpy_arr(defSlider[0].other, "-3.1416 3.1416");
      //   }

      // add and count
      mjui_add(&sim->ui1, defSlider);
      itemcnt++;
    }
  }
}

// make control section of UI
void MakeControlSection(mujoco_with_ros2::MujocoWithROS2Render* sim)
{
  mjuiDef defControl[] = {{mjITEM_SECTION, "Control", mjPRESERVE, nullptr, "AC"},
                          {mjITEM_BUTTON, "Clear all", 2},
                          {mjITEM_END}};
  mjuiDef defSlider[]  = {{mjITEM_SLIDERNUM, "", 2, nullptr, "0 1"}, {mjITEM_END}};

  // add section
  mjui_add(&sim->ui1, defControl);

  // add controls, exit if UI limit reached (Clear button already added)
  int itemcnt = 1;
  for (int i = 0; i < sim->actuator_ctrlrange_.size() && itemcnt < mjMAXUIITEM; i++)
  {
    // skip if actuator vis group is disabled
    int group = sim->actuator_group_[i];
    if (!sim->opt.actuatorgroup[mjMAX(0, mjMIN(mjNGROUP - 1, group))])
    {
      continue;
    }
    // grey out if actuator group is disabled
    if (group >= 0 && group <= 30 && sim->m_->opt.disableactuator & (1 << group))
    {
      defSlider[0].state = 0;
    }
    else
    {
      defSlider[0].state = 2;
    }

    // set data and name

    defSlider[0].pdata = &sim->ctrl_[i];

    if (!sim->actuator_names_[i].empty())
    {
      mju::strcpy_arr(defSlider[0].name, sim->actuator_names_[i].c_str());
    }
    else
    {
      mju::sprintf_arr(defSlider[0].name, "control %d", i);
    }

    // set range
    // if (sim->actuator_ctrlrange_[i].has_value())
    //   mju::sprintf_arr(defSlider[0].other, "%.4g %.4g",
    //                    sim->actuator_ctrlrange_[i]->first, sim->actuator_ctrlrange_[i]->second);
    // else {
    mju::strcpy_arr(defSlider[0].other, "-1 1");
    // }

    // add and count
    mjui_add(&sim->ui1, defSlider);
    itemcnt++;
  }
}

// // make equality section of UI
// void MakeEqualitySection(mujoco_with_ros2::MujocoWithROS2Render* sim) {
//   mjuiDef defEquality[] = {
//     {mjITEM_SECTION, "Equality", mjPRESERVE, nullptr, "AE"},
//     {mjITEM_END}
//   };
//   mjuiDef defCheckBox[] = {
//     {mjITEM_CHECKBYTE, "", 2, nullptr, ""},
//     {mjITEM_END}
//   };

//   // add section
//   mjui_add(&sim->ui1, defEquality);

//   // add equalities, exit if UI limit reached
//   for (int i= 0; i < sim->m_->neq && i<mjMAXUIITEM; i++) {
//     // set data
//     defCheckBox[0].pdata = &sim->d_->eq_active[i];

//     // set name
//     if (!sim->equality_names_[i].empty()) {
//       mju::strcpy_arr(defCheckBox[0].name, sim->equality_names_[i].c_str());
//     } else {
//       mju::sprintf_arr(defCheckBox[0].name, "equality %d", i);
//     }

// mjui_add(&sim->ui1, defCheckBox);
// }
// }

// make model-dependent UI sections
void MakeUiSections(mujoco_with_ros2::MujocoWithROS2Render* sim, const mjModel* m, const mjData* d)
{
  // clear model-dependent sections of UI
  sim->ui0.nsect = SECT_PHYSICS;
  sim->ui1.nsect = 0;

  // make
  MakePhysicsSection(sim);
  MakeRenderingSection(sim, m);
  MakeVisualizationSection(sim, m);
  MakeGroupSection(sim);
  MakeJointSection(sim);
  MakeControlSection(sim);
  //   MakeEqualitySection(sim);
}

//---------------------------------- utility functions ---------------------------------------------

// align and scale view
void AlignAndScaleView(mujoco_with_ros2::MujocoWithROS2Render* sim, const mjModel* m)
{
  //   // if the id is valid, use the initial fixed camera
  //   if (m->vis.global.cameraid >= 0 && m->vis.global.cameraid < m->ncam)
  //   {
  //     sim->cam.fixedcamid = m->vis.global.cameraid;
  //     sim->cam.type       = mjCAMERA_FIXED;
  //   }

  //   // otherwise use default free camera
  //   else
  //   {
  mjv_defaultFreeCamera(m, &sim->cam);
  //   }
}


// copy state to clipboard as key
void CopyKey(mujoco_with_ros2::MujocoWithROS2Render* sim,
             const mjModel* m,
             const mjData* d,
             bool fp)
{
  char clipboard[5000] = "<key\n";
  char buf[200];
  const char p_regular[] = "%g";
  const char p_full[]    = "%-22.16g";
  const char* format     = fp ? p_full : p_regular;

  // time
  mju::strcat_arr(clipboard, "  time=\"");
  mju::sprintf_arr(buf, format, d->time);
  mju::strcat_arr(clipboard, buf);

  // qpos
  mju::strcat_arr(clipboard, "\"\n  qpos=\"");
  for (int i = 0; i < m->nq; i++)
  {
    mju::sprintf_arr(buf, format, d->qpos[i]);
    if (i < m->nq - 1)
      mju::strcat_arr(buf, " ");
    mju::strcat_arr(clipboard, buf);
  }

  // qvel
  mju::strcat_arr(clipboard, "\"\n  qvel=\"");
  for (int i = 0; i < m->nv; i++)
  {
    mju::sprintf_arr(buf, format, d->qvel[i]);
    if (i < m->nv - 1)
      mju::strcat_arr(buf, " ");
    mju::strcat_arr(clipboard, buf);
  }

  // act
  if (m->na > 0)
  {
    mju::strcat_arr(clipboard, "\"\n  act=\"");
    for (int i = 0; i < m->na; i++)
    {
      mju::sprintf_arr(buf, format, d->act[i]);
      if (i < m->na - 1)
        mju::strcat_arr(buf, " ");
      mju::strcat_arr(clipboard, buf);
    }
  }

  // ctrl
  if (m->nu > 0)
  {
    mju::strcat_arr(clipboard, "\"\n  ctrl=\"");
    for (int i = 0; i < m->nu; i++)
    {
      mju::sprintf_arr(buf, format, d->ctrl[i]);
      if (i < m->nu - 1)
        mju::strcat_arr(buf, " ");
      mju::strcat_arr(clipboard, buf);
    }
  }

  if (m->nmocap > 0)
  {
    // mocap_pos
    mju::strcat_arr(clipboard, "\"\n  mpos=\"");
    for (int i = 0; i < 3 * m->nmocap; i++)
    {
      mju::sprintf_arr(buf, format, d->mocap_pos[i]);
      if (i < 3 * m->nmocap - 1)
        mju::strcat_arr(buf, " ");
      mju::strcat_arr(clipboard, buf);
    }

    // mocap_quat
    mju::strcat_arr(clipboard, "\"\n  mquat=\"");
    for (int i = 0; i < 4 * m->nmocap; i++)
    {
      mju::sprintf_arr(buf, format, d->mocap_quat[i]);
      if (i < 4 * m->nmocap - 1)
        mju::strcat_arr(buf, " ");
      mju::strcat_arr(clipboard, buf);
    }
  }

  mju::strcat_arr(clipboard, "\"\n/>");

  // copy to clipboard
  sim->platform_ui->SetClipboardString(clipboard);
}

// millisecond timer, for MuJoCo built-in profiler
mjtNum Timer()
{
  static auto start = mujoco_with_ros2::MujocoWithROS2Render::Clock::now();
  auto elapsed      = Milliseconds(mujoco_with_ros2::MujocoWithROS2Render::Clock::now() - start);
  return elapsed.count();
}

// clear all times
void ClearTimers(mjData* d)
{
  for (int i = 0; i < mjNTIMER; i++)
  {
    d->timer[i].duration = 0;
    d->timer[i].number   = 0;
  }
}

// copy current camera to clipboard as MJCF specification
void CopyCamera(mujoco_with_ros2::MujocoWithROS2Render* sim)
{
  mjvGLCamera* camera = sim->scn.camera;

  char clipboard[500];
  mjtNum cam_right[3];
  mjtNum cam_forward[3];
  mjtNum cam_up[3];

  // get camera spec from the GLCamera
  mju_f2n(cam_forward, camera[0].forward, 3);
  mju_f2n(cam_up, camera[0].up, 3);
  mju_cross(cam_right, cam_forward, cam_up);

  // make MJCF camera spec
  mju::sprintf_arr(clipboard,
                   "<camera pos=\"%.3f %.3f %.3f\" xyaxes=\"%.3f %.3f %.3f %.3f %.3f %.3f\"/>\n",
                   (camera[0].pos[0] + camera[1].pos[0]) / 2,
                   (camera[0].pos[1] + camera[1].pos[1]) / 2,
                   (camera[0].pos[2] + camera[1].pos[2]) / 2,
                   cam_right[0],
                   cam_right[1],
                   cam_right[2],
                   camera[0].up[0],
                   camera[0].up[1],
                   camera[0].up[2]);

  // copy spec into clipboard
  sim->platform_ui->SetClipboardString(clipboard);
}

// update UI 0 when MuJoCo structures change (except for joint sliders)
void UpdateSettings(mujoco_with_ros2::MujocoWithROS2Render* sim, const mjModel* m)
{
  // physics flags
  for (int i = 0; i < mjNDISABLE; i++)
  {
    int new_value = ((m->opt.disableflags & (1 << i)) != 0);
    if (sim->disable[i] != new_value)
    {
      sim->disable[i]                 = new_value;
      sim->pending_.ui_update_physics = true;
    }
  }
  for (int i = 0; i < mjNENABLE; i++)
  {
    int new_value = ((m->opt.enableflags & (1 << i)) != 0);
    if (sim->enable[i] != new_value)
    {
      sim->enable[i]                  = new_value;
      sim->pending_.ui_update_physics = true;
    }
  }
  for (int i = 0; i < mjNGROUP; i++)
  {
    int enabled = ((m->opt.disableactuator & (1 << i)) == 0);
    if (sim->enableactuator[i] != enabled)
    {
      sim->enableactuator[i]          = enabled;
      sim->pending_.ui_update_physics = true;
      sim->pending_.ui_remake_ctrl    = true;
    }
  }

  // camera
  int old_camera = sim->camera;
  if (sim->cam.type == mjCAMERA_FIXED)
  {
    sim->camera = 2 + sim->cam.fixedcamid;
  }
  else if (sim->cam.type == mjCAMERA_TRACKING)
  {
    sim->camera = 1;
  }
  else
  {
    sim->camera = 0;
  }
  if (old_camera != sim->camera)
  {
    sim->pending_.ui_update_rendering = true;
  }
}

// Compute suitable font scale.
int ComputeFontScale(const mujoco_with_ros2::MujocoWithRos2UIAdapter& platform_ui)
{
  // compute framebuffer-to-window ratio
  auto [buf_width, buf_height] = platform_ui.GetFramebufferSize();
  auto [win_width, win_height] = platform_ui.GetWindowSize();
  double b2w                   = static_cast<double>(buf_width) / win_width;

  // compute PPI
  double PPI = b2w * platform_ui.GetDisplayPixelsPerInch();

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


//---------------------------------- UI handlers ---------------------------------------------------

// // determine enable/disable item state given category
// int UiPredicate(int category, void* userdata)
// {
//   mujoco_with_ros2::MujocoWithROS2Render* sim =
//     static_cast<mujoco_with_ros2::MujocoWithROS2Render*>(userdata);

//   switch (category)
//   {
//     // case 2: // require model
//     //   return sim->m_ ;

//     // case 3: // require model and nkey
//     //   return (sim->m_ ) && sim->nkey_;

//     case 4: // require model and paused
//       return sim->m_ && !sim->run;

//     case 5: // require model and fully managed mode
//       return sim->m_;

//     default:
//       return 1;
//   }
// }

// set window layout
void UiLayout(mjuiState* state)
{
  mujoco_with_ros2::MujocoWithROS2Render* sim =
    static_cast<mujoco_with_ros2::MujocoWithROS2Render*>(state->userdata);
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

// handle UI event
void UiEvent(mjuiState* state)
{
  mujoco_with_ros2::MujocoWithROS2Render* sim =
    static_cast<mujoco_with_ros2::MujocoWithROS2Render*>(state->userdata);

  // call UI 0 if event is directed to it
  if ((state->dragrect == sim->ui0.rectid) ||
      (state->dragrect == 0 && state->mouserect == sim->ui0.rectid) || state->type == mjEVENT_KEY)
  {
    // process UI event
    mjuiItem* it = mjui_event(&sim->ui0, state, &sim->platform_ui->mjr_context());

    // file section
    if (it && it->sectionid == SECT_FILE)
    {
      switch (it->itemid)
      {
        case 0: // Save xml
          sim->pending_.save_xml = GetSavePath("mjmodel.xml");
          break;

        case 1: // Save mjb
          sim->pending_.save_mjb = GetSavePath("mjmodel.mjb");
          break;

        case 2: // Print model
          sim->pending_.print_model = GetSavePath("MJMODEL.TXT");
          break;

        case 3: // Print data
          sim->pending_.print_data = GetSavePath("MJDATA.TXT");
          break;

        case 4: // Quit
          sim->exitrequest.store(1);
          break;

        case 5: // Screenshot
          sim->screenshotrequest.store(true);
          break;
      }
    }

    // option section
    else if (it && it->sectionid == SECT_OPTION)
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
        mjr_changeFont(50 * (sim->font + 1), &sim->platform_ui->mjr_context());
      }
      else if (it->pdata == &sim->fullscreen)
      {
        sim->platform_ui->ToggleFullscreen();
      }
      else if (it->pdata == &sim->vsync)
      {
        sim->platform_ui->SetVSync(sim->vsync);
      }

      // modify UI
      UiModify(&sim->ui0, state, &sim->platform_ui->mjr_context());
      UiModify(&sim->ui1, state, &sim->platform_ui->mjr_context());
    }

    // simulation section
    else if (it && it->sectionid == SECT_SIMULATION)
    {
      switch (it->itemid)
      {
        case 1: // Reset
          sim->pending_.reset = true;
          break;

        case 2: // Reload
          sim->uiloadrequest.fetch_add(1);
          break;

        case 3: // Align
          sim->pending_.align = true;
          break;

        case 4: // Copy key
          sim->pending_.copy_key                = true;
          sim->pending_.copy_key_full_precision = sim->platform_ui->IsShiftKeyPressed();
          break;

        case 5: // Adjust key
        case 6: // Load key
          sim->pending_.load_key = true;
          break;

        case 7: // Save key
          sim->pending_.save_key = true;
          break;

        case 11: // History scrubber
          sim->run                        = 0;
          sim->pending_.load_from_history = true;
          mjui0_update_section(sim, SECT_SIMULATION);
          break;
      }
    }

    // physics section
    else if (it && it->sectionid == SECT_PHYSICS && sim->m_)
    {
      mjOption* opt = &sim->m_->opt;

      // update disable flags in mjOption
      opt->disableflags = 0;
      for (int i = 0; i < mjNDISABLE; i++)
      {
        if (sim->disable[i])
        {
          opt->disableflags |= (1 << i);
        }
      }

      // update enable flags in mjOption
      opt->enableflags = 0;
      for (int i = 0; i < mjNENABLE; i++)
      {
        if (sim->enable[i])
        {
          opt->enableflags |= (1 << i);
        }
      }

      // update disableactuator bitflag in mjOption
      bool group_changed = false;
      for (int i = 0; i < mjNGROUP; i++)
      {
        if ((!sim->enableactuator[i]) != (opt->disableactuator & (1 << i)))
        {
          group_changed = true;
          if (!sim->enableactuator[i])
          {
            // disable actuator group i
            opt->disableactuator |= (1 << i);
          }
          else
          {
            // enable actuator group i
            opt->disableactuator &= ~(1 << i);
          }
        }
      }

      // remake control section if actuator disable group changed
      if (group_changed)
      {
        sim->pending_.ui_remake_ctrl = true;
      }
    }

    // rendering section
    else if (it && it->sectionid == SECT_RENDERING)
    {
      // set camera in mjvCamera
      if (sim->camera == 0)
      {
        sim->cam.type = mjCAMERA_FREE;
      }
      else if (sim->camera == 1)
      {
        if (sim->pert.select > 0)
        {
          sim->cam.type        = mjCAMERA_TRACKING;
          sim->cam.trackbodyid = sim->pert.select;
          sim->cam.fixedcamid  = -1;
        }
        else
        {
          sim->cam.type = mjCAMERA_FREE;
          sim->camera   = 0;
          mjui0_update_section(sim, SECT_RENDERING);
        }
      }
      else
      {
        sim->cam.type       = mjCAMERA_FIXED;
        sim->cam.fixedcamid = sim->camera - 2;
      }
      // copy camera spec to clipboard (as MJCF element)
      if (it->itemid == 3)
      {
        CopyCamera(sim);
      }
    }

    // visualization section
    else if (it && it->sectionid == SECT_VISUALIZATION)
    {
      if (!mju::strcmp_arr(it->name, "Align"))
      {
        sim->pending_.align = true;
      }
    }

    // group section
    else if (it && it->sectionid == SECT_GROUP)
    {
      // remake joint section if joint group changed
      if (it->name[0] == 'J' && it->name[1] == 'o')
      {
        sim->ui1.nsect = SECT_JOINT;
        MakeJointSection(sim);
        sim->ui1.nsect = NSECT1;
        UiModify(&sim->ui1, state, &sim->platform_ui->mjr_context());
      }

      // remake control section if actuator group changed
      if (it->name[0] == 'A' && it->name[1] == 'c')
      {
        sim->pending_.ui_remake_ctrl = true;
      }
    }

    // stop if UI processed event
    if (it != nullptr || (state->type == mjEVENT_KEY && state->key == 0))
    {
      return;
    }
  }

  // call UI 1 if event is directed to it
  if ((state->dragrect == sim->ui1.rectid) ||
      (state->dragrect == 0 && state->mouserect == sim->ui1.rectid) || state->type == mjEVENT_KEY)
  {
    // process UI event
    mjuiItem* it = mjui_event(&sim->ui1, state, &sim->platform_ui->mjr_context());

    // control section
    if (it && it->sectionid == SECT_CONTROL)
    {
      // clear controls
      if (it->itemid == 0)
      {
        sim->pending_.zero_ctrl = true;
      }
    }

    // stop if UI processed event
    if (it != nullptr || (state->type == mjEVENT_KEY && state->key == 0))
    {
      return;
    }
  }

  // shortcut not handled by UI
  if (state->type == mjEVENT_KEY && state->key != 0)
  {
    switch (state->key)
    {
      case ' ': // Mode
        if (sim->m_)
        {
          sim->run         = 1 - sim->run;
          sim->pert.active = 0;

          if (sim->run)
            sim->scrub_index = 0; // reset scrubber

          mjui0_update_section(sim, -1);
        }
        break;

      case mjKEY_RIGHT: // step forward
        if (sim->m_ && !sim->run)
        {
          ClearTimers(sim->d_);

          // currently in scrubber: increment scrub, load state, update slider UI
          if (sim->scrub_index < 0)
          {
            sim->scrub_index++;
            sim->pending_.load_from_history = true;
            mjui0_update_section(sim, SECT_SIMULATION);
          }

          // not in scrubber: step, add to history buffer
          else
          {
            mj_step(sim->m_, sim->d_);
            // sim->AddToHistory();
          }

          //   UpdateProfiler(sim, sim->m_, sim->d_);
          //   UpdateSensor(sim, sim->m_, sim->d_);
          UpdateSettings(sim, sim->m_);
        }
        break;

      case mjKEY_LEFT: // step backward
        if (sim->m_)
        {
          sim->run = 0;
          ClearTimers(sim->d_);

          // decrement scrub, load state
          //   sim->scrub_index                = mjMAX(sim->scrub_index - 1, 1 - sim->nhistory_);
          sim->pending_.load_from_history = true;

          // update slider UI, profiler, sensor
          mjui0_update_section(sim, SECT_SIMULATION);
          //   UpdateProfiler(sim, sim->m_, sim->d_);
          //   UpdateSensor(sim, sim->m_, sim->d_);
        }
        break;

      case mjKEY_PAGE_UP: // select parent body
        if (sim->m_ && sim->pert.select > 0)
        {
          sim->pert.select     = sim->body_parentid_[sim->pert.select];
          sim->pert.flexselect = -1;
          sim->pert.skinselect = -1;

          // stop perturbation if world reached
          if (sim->pert.select <= 0)
          {
            sim->pert.active = 0;
          }
        }

        break;

      case ']': // cycle up fixed cameras
        if ((sim->m_) && sim->ncam_)
        {
          sim->cam.type = mjCAMERA_FIXED;
          // camera = {0 or 1} are reserved for the free and tracking cameras
          if (sim->camera < 2 || sim->camera == 2 + sim->ncam_ - 1)
          {
            sim->camera = 2;
          }
          else
          {
            sim->camera += 1;
          }
          sim->cam.fixedcamid = sim->camera - 2;
          mjui0_update_section(sim, SECT_RENDERING);
        }
        break;

      case '[': // cycle down fixed cameras
        if ((sim->m_) && sim->ncam_)
        {
          sim->cam.type = mjCAMERA_FIXED;
          // camera = {0 or 1} are reserved for the free and tracking cameras
          if (sim->camera <= 2)
          {
            sim->camera = 2 + sim->ncam_ - 1;
          }
          else
          {
            sim->camera -= 1;
          }
          sim->cam.fixedcamid = sim->camera - 2;
          mjui0_update_section(sim, SECT_RENDERING);
        }
        break;

      case mjKEY_F6: // cycle frame visualisation
        if (sim->m_)
        {
          sim->opt.frame = (sim->opt.frame + 1) % mjNFRAME;
          mjui0_update_section(sim, SECT_RENDERING);
        }
        break;

      case mjKEY_F7: // cycle label visualisation
        if (sim->m_)
        {
          sim->opt.label = (sim->opt.label + 1) % mjNLABEL;
          mjui0_update_section(sim, SECT_RENDERING);
        }
        break;

      case mjKEY_ESCAPE: // free camera
        sim->cam.type = mjCAMERA_FREE;
        sim->camera   = 0;
        mjui0_update_section(sim, SECT_RENDERING);
        break;

      case '-': // slow down
        {
        int numclicks = sizeof(sim->percentRealTime) / sizeof(sim->percentRealTime[0]);
        if (sim->real_time_index < numclicks - 1 && !state->shift)
        {
          sim->real_time_index++;
          sim->speed_changed = true;
        }
      }
        break;

      case '=': // speed up
        if (sim->real_time_index > 0 && !state->shift)
        {
          sim->real_time_index--;
          sim->speed_changed = true;
        }
        break;

      case mjKEY_TAB: // toggle left/right UI
        if (!state->shift)
        {
          // toggle left UI
          sim->ui0_enable = !sim->ui0_enable;
          UiModify(&sim->ui0, state, &sim->platform_ui->mjr_context());
        }
        else
        {
          // toggle right UI
          sim->ui1_enable = !sim->ui1_enable;
          UiModify(&sim->ui1, state, &sim->platform_ui->mjr_context());
        }
        break;
    }

    return;
  }

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
      if (newperturb && !sim->pert.active)
      {
        sim->pending_.newperturb = newperturb;
      }
    }

    // handle double-click
    if (state->doubleclick && (sim->m_))
    {
      sim->pending_.select = true;
      std::memcpy(&sim->pending_.select_state, state, sizeof(sim->pending_.select_state));

      // stop perturbation on select
      sim->pert.active         = 0;
      sim->pending_.newperturb = 0;
    }

    return;
  }

  // 3D release
  if (state->type == mjEVENT_RELEASE && state->dragrect == 3 && (sim->m_))
  {
    // stop perturbation
    sim->pert.active         = 0;
    sim->pending_.newperturb = 0;
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
  if (state->type == mjEVENT_FILESDROP && state->dropcount > 0 )
  {
    while (sim->droploadrequest.load())
    {
    }
    mju::strcpy_arr(sim->dropfilename, state->droppaths[0]);
    sim->droploadrequest.store(true);
    return;
  }

  // Redraw
  if (state->type == mjEVENT_REDRAW)
  {
    sim->Render();
    return;
  }
}
} // namespace mujoco_with_ros2

namespace mujoco_with_ros2 {
namespace mju = ::mujoco_with_ros2::sample_util;

MujocoWithROS2Render::MujocoWithROS2Render(std::unique_ptr<MujocoWithRos2UIAdapter> platform_ui,
                                           mjvCamera* cam,
                                           mjvOption* opt,
                                           mjvPerturb* pert)

  : cam(*cam)
  , opt(*opt)
  , pert(*pert)
  , platform_ui(std::move(platform_ui))
  , uistate(this->platform_ui->state())
{
  mjv_defaultScene(&scn);
}


//------------------------- Synchronize render and physics threads ---------------------------------

// operations which require holding the mutex, prevents racing with physics thread
void MujocoWithROS2Render::Sync(bool state_only)
{
  MutexLock lock(this->mtx);

  if (!m_)
  {
    return;
  }
  if (this->exitrequest.load())
  {
    return;
  }

//   bool update_profiler = this->profiler;
//   bool update_sensor   = this->sensor;

//   for (int i = 0; i < m_->njnt; ++i)
//   {
//     std::pair<mjtNum, mjtNum> range;
//     if (m_->jnt_limited[i])
//     {
//       range.emplace(m_->jnt_range[2 * i], m_->jnt_range[2 * i + 1]);
//     }
//     if (jnt_range_[i] != range)
//     {
//       pending_.ui_update_joint = true;
//       jnt_range_[i].swap(range);
//     }
//   }

//   for (int i = 0; i < m_->nu; ++i)
//   {
//     std::optional<std::pair<mjtNum, mjtNum> > range;
//     if (m_->actuator_ctrllimited[i])
//     {
//       range.emplace(m_->actuator_ctrlrange[2 * i], m_->actuator_ctrlrange[2 * i + 1]);
//     }
//     if (actuator_ctrlrange_[i] != range)
//     {
//       pending_.ui_remake_ctrl = true;
//       actuator_ctrlrange_[i].swap(range);
//     }
//   }

//   for (int i = 0; i < m_->nq; ++i)
//   {
//     if (qpos_[i] != qpos_prev_[i])
//     {
//       d_->qpos[i] = qpos_[i];
//     }
//     else
//     {
//       qpos_[i] = d_->qpos[i];
//     }
//     if (qpos_prev_[i] != qpos_[i])
//     {
//       pending_.ui_update_joint = true;
//       qpos_prev_[i]            = qpos_[i];
//     }
//   }

//   for (int i = 0; i < m_->nu; ++i)
//   {
//     if (ctrl_[i] != ctrl_prev_[i])
//     {
//       d_->ctrl[i] = ctrl_[i];
//     }
//     else
//     {
//       ctrl_[i] = d_->ctrl[i];
//     }
//     if (ctrl_prev_[i] != ctrl_[i])
//     {
//       pending_.ui_update_ctrl = true;
//       ctrl_prev_[i]           = ctrl_[i];
//     }
//   }

//   for (int i = 0; i < m_->neq; ++i)
//   {
//     if (eq_active_[i] != eq_active_prev_[i])
//     {
//       d_->eq_active[i] = eq_active_[i];
//     }
//     else
//     {
//       eq_active_[i] = d_->eq_active[i];
//     }
//     if (eq_active_prev_[i] != eq_active_[i])
//     {
//       pending_.ui_update_equality = true;
//       eq_active_prev_[i]          = eq_active_[i];
//     }
//   }

//   `
}

//------------------------- Tell the render thread to load a file and wait -------------------------
// void MujocoWithROS2Render::LoadMessage(const char* displayed_filename)
// {
//   mju::strcpy_arr(this->filename, displayed_filename);

//   {
//     MutexLock lock(mtx);
//     this->loadrequest = 3;
//   }
// }

// void MujocoWithROS2Render::Load(mjModel* m, mjData* d, const char* displayed_filename)
// {
//   this->mnew_ = m;
//   this->dnew_ = d;
//   mju::strcpy_arr(this->filename, displayed_filename);

//   {
//     MutexLock lock(mtx);
//     this->loadrequest = 2;

//     // Wait for the render thread to be done loading
//     // so that we know the old model and data's memory can
//     // be free'd by the other thread (sometimes python)
//     cond_loadrequest.wait(lock, [this]() { return this->loadrequest == 0; });
//   }
// }

// void MujocoWithROS2Render::LoadMessageClear(void)
// {
//   {
//     MutexLock lock(mtx);
//     this->loadrequest = 0;
//   }
// }




//------------------------------------------- rendering --------------------------------------------

// render the ui to the window
void MujocoWithROS2Render::Render()
{
  // update rendering context buffer size if required
  if (this->platform_ui->EnsureContextSize())
  {
    UiModify(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
    UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
  }

  // get 3D rectangle and reduced for profiler
  mjrRect rect      = this->uistate.rect[3];
  mjrRect smallrect = rect;
  if (this->profiler)
  {
    smallrect.width = rect.width - rect.width / 4;
  }

  // no model
  if (!this->m_)
  {
    // blank screen
    mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);

    // label
    if (this->loadrequest)
    {
      mjr_overlay(mjFONT_BIG,
                  mjGRID_TOP,
                  smallrect,
                  "LOADING...",
                  nullptr,
                  &this->platform_ui->mjr_context());
    }
    else
    {
      char intro_message[MujocoWithROS2Render::kMaxFilenameLength];
      mju::sprintf_arr(
        intro_message, "MuJoCo version %s\nDrag-and-drop model file here", mj_versionString());
      mjr_overlay(
        mjFONT_NORMAL, mjGRID_TOPLEFT, rect, intro_message, 0, &this->platform_ui->mjr_context());
    }

    // show last loading error
    if (this->load_error[0])
    {
      mjr_overlay(mjFONT_NORMAL,
                  mjGRID_BOTTOMLEFT,
                  rect,
                  this->load_error,
                  0,
                  &this->platform_ui->mjr_context());
    }

    // render uis
    if (this->ui0_enable)
    {
      mjui_render(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
    }
    if (this->ui1_enable)
    {
      mjui_render(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
    }

    // finalize
    this->platform_ui->SwapBuffers();

    return;
  }

  // update UI sections from last sync
  if (pending_.ui_update_simulation)
  {
    if (this->ui0_enable && this->ui0.sect[SECT_SIMULATION].state)
    {
      mjui0_update_section(this, SECT_SIMULATION);
    }
    pending_.ui_update_simulation = false;
  }

  if (this->ui0_enable && this->ui0.sect[SECT_WATCH].state)
  {
    mjui0_update_section(this, SECT_WATCH);
  }

  if (pending_.ui_update_physics)
  {
    if (this->ui0_enable && this->ui0.sect[SECT_PHYSICS].state)
    {
      mjui0_update_section(this, SECT_PHYSICS);
    }
    pending_.ui_update_physics = false;
  }

  if (pending_.ui_update_visualization)
  {
    if (this->ui0_enable && this->ui0.sect[SECT_VISUALIZATION].state)
    {
      mjui0_update_section(this, SECT_VISUALIZATION);
    }
    pending_.ui_update_visualization = false;
  }

  // if (is_passive_)
  // {
  //   if (this->ui0_enable && this->ui0.sect[SECT_RENDERING].state &&
  //       (cam_prev_.type != cam.type || cam_prev_.fixedcamid != cam.fixedcamid ||
  //        cam_prev_.trackbodyid != cam.trackbodyid || opt_prev_.label != opt.label ||
  //        opt_prev_.frame != opt.frame || IsDifferent(opt_prev_.flags, opt.flags)))
  //   {
  //     pending_.ui_update_rendering = true;
  //   }

  //   if (this->ui0_enable && this->ui0.sect[SECT_RENDERING].state &&
  //       (IsDifferent(opt_prev_.geomgroup, opt.geomgroup) ||
  //        IsDifferent(opt_prev_.sitegroup, opt.sitegroup) ||
  //        IsDifferent(opt_prev_.jointgroup, opt.jointgroup) ||
  //        IsDifferent(opt_prev_.tendongroup, opt.tendongroup) ||
  //        IsDifferent(opt_prev_.actuatorgroup, opt.actuatorgroup) ||
  //        IsDifferent(opt_prev_.flexgroup, opt.flexgroup) ||
  //        IsDifferent(opt_prev_.skingroup, opt.skingroup)))
  //   {
  //     mjui0_update_section(this, SECT_GROUP);
  //   }

  //   opt_prev_ = opt;
  //   cam_prev_ = cam;
  // }

  if (pending_.ui_update_rendering)
  {
    if (this->ui0_enable && this->ui0.sect[SECT_RENDERING].state)
    {
      mjui0_update_section(this, SECT_RENDERING);
    }
    pending_.ui_update_rendering = false;
  }

  if (pending_.ui_update_joint)
  {
    if (this->ui1_enable && this->ui1.sect[SECT_JOINT].state)
    {
      mjui_update(SECT_JOINT, -1, &this->ui1, &this->uistate, &this->platform_ui->mjr_context());
    }
    pending_.ui_update_joint = false;
  }

  if (pending_.ui_remake_ctrl)
  {
    if (this->ui1_enable && this->ui1.sect[SECT_CONTROL].state)
    {
      this->ui1.nsect = SECT_CONTROL;
      MakeControlSection(this);
      this->ui1.nsect = NSECT1;
      UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
    }
    pending_.ui_remake_ctrl = false;
  }

  if (pending_.ui_update_ctrl)
  {
    if (this->ui1_enable && this->ui1.sect[SECT_CONTROL].state)
    {
      mjui_update(SECT_CONTROL, -1, &this->ui1, &this->uistate, &this->platform_ui->mjr_context());
    }
    pending_.ui_update_ctrl = false;
  }

  if (pending_.ui_update_equality)
  {
    if (this->ui1_enable && this->ui1.sect[SECT_EQUALITY].state)
    {
      mjui_update(SECT_EQUALITY, -1, &this->ui1, &this->uistate, &this->platform_ui->mjr_context());
    }
    pending_.ui_update_equality = false;
  }

  // render scene
  mjr_render(rect, &this->scn, &this->platform_ui->mjr_context());

  // show last loading error
  if (this->load_error[0])
  {
    mjr_overlay(mjFONT_NORMAL,
                mjGRID_BOTTOMLEFT,
                rect,
                this->load_error,
                0,
                &this->platform_ui->mjr_context());
  }

  // show pause/loading label
  if (!this->run || this->loadrequest)
  {
    char label[30] = {'\0'};
    if (this->loadrequest)
    {
      std::snprintf(label, sizeof(label), "LOADING...");
    }
    else if (this->scrub_index == 0)
    {
      std::snprintf(label, sizeof(label), "PAUSE");
    }
    else
    {
      std::snprintf(label, sizeof(label), "PAUSE (%d)", this->scrub_index);
    }
    mjr_overlay(
      mjFONT_BIG, mjGRID_TOP, smallrect, label, nullptr, &this->platform_ui->mjr_context());
  }

  // get desired and actual percent-of-real-time
  float desiredRealtime = this->percentRealTime[this->real_time_index];
  float actualRealtime  = 100 / this->measured_slowdown;

  // if running, check for misalignment of more than 10%
  float realtime_offset = mju_abs(actualRealtime - desiredRealtime);
  bool misaligned       = this->run && realtime_offset > 0.1 * desiredRealtime;

  // make realtime overlay label
  char rtlabel[30] = {'\0'};
  if (desiredRealtime != 100.0 || misaligned)
  {
    // print desired realtime
    int labelsize = std::snprintf(rtlabel, sizeof(rtlabel), "%g%%", desiredRealtime);

    // if misaligned, append to label
    if (misaligned)
    {
      std::snprintf(
        rtlabel + labelsize, sizeof(rtlabel) - labelsize, " (%-4.1f%%)", actualRealtime);
    }
  }

  // show real-time overlay
  if (rtlabel[0])
  {
    mjr_overlay(
      mjFONT_BIG, mjGRID_TOPLEFT, smallrect, rtlabel, nullptr, &this->platform_ui->mjr_context());
  }

  // show ui 0
  if (this->ui0_enable)
  {
    mjui_render(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
  }

  // show ui 1
  if (this->ui1_enable)
  {
    mjui_render(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
  }

  // show help
  if (this->help)
  {
    mjr_overlay(mjFONT_NORMAL,
                mjGRID_TOPLEFT,
                rect,
                help_title,
                help_content,
                &this->platform_ui->mjr_context());
  }

  // show info
  if (this->info)
  {
    mjr_overlay(mjFONT_NORMAL,
                mjGRID_BOTTOMLEFT,
                rect,
                this->info_title,
                this->info_content,
                &this->platform_ui->mjr_context());
  }

  // // show profiler
  // if (this->profiler)
  // {
  //   ShowProfiler(this, rect);
  // }

  // // show sensor
  // if (this->sensor)
  // {
  //   ShowSensor(this, smallrect);
  // }

  // take screenshot, save to file
  // if (this->screenshotrequest.exchange(false))
  // {
  //   const unsigned int h = uistate.rect[0].height;
  //   const unsigned int w = uistate.rect[0].width;
  //   std::unique_ptr<unsigned char[]> rgb(new unsigned char[3 * w * h]);
  //   if (!rgb)
  //   {
  //     mju_error("could not allocate buffer for screenshot");
  //   }
  //   mjr_readPixels(rgb.get(), nullptr, uistate.rect[0], &this->platform_ui->mjr_context());

  //   // flip up-down
  //   for (int r = 0; r < h / 2; ++r)
  //   {
  //     unsigned char* top_row    = &rgb[3 * w * r];
  //     unsigned char* bottom_row = &rgb[3 * w * (h - 1 - r)];
  //     std::swap_ranges(top_row, top_row + 3 * w, bottom_row);
  //   }

  //   // save as PNG
  //   // TODO(b/241577466): Parse the stem of the filename and use a .PNG extension.
  //   // Unfortunately, if we just yank ".xml"/".mjb" from the filename and append .PNG, the macOS
  //   // file dialog does not automatically open that location. Thus, we defer to a default
  //   // "screenshot.png" for now.
  //   const std::string path = GetSavePath("screenshot.png");
  //   if (!path.empty())
  //   {
  //     // if (lodepng::encode(path, rgb.get(), w, h, LCT_RGB))
  //     // {
  //     //   mju_error("could not save screenshot");
  //     // }
  //     // else
  //     // {
  //       std::printf("saved screenshot: %s\n", path.c_str());
  //     // }
  //   }
  // }

  // // user figures
  // if (this->newfigurerequest.load() == 1)
  // {
  //   this->user_figures_.clear();
  //   std::swap(this->user_figures_, this->user_figures_new_);
  //   int value = 1;
  //   this->newfigurerequest.compare_exchange_strong(value, 0);
  // }
  // for (auto& [viewport, figure] : this->user_figures_)
  // {
  //   ShowFigure(this, viewport, &figure);
  // }

  // // overlay text
  // if (this->newtextrequest.load() == 1)
  // {
  //   this->user_texts_.clear();
  //   std::swap(this->user_texts_, this->user_texts_new_);
  //   int value = 1;
  //   this->newtextrequest.compare_exchange_strong(value, 0);
  // }
  // for (auto& [font, gridpos, text1, text2] : this->user_texts_)
  // {
  //   ShowOverlayText(this, rect, font, gridpos, text1, text2);
  // }

  // // user images
  // if (this->newimagerequest.load() == 1)
  // {
  //   this->user_images_.clear();
  //   std::swap(this->user_images_, this->user_images_new_);
  //   int value = 1;
  //   this->newimagerequest.compare_exchange_strong(value, 0);
  // }
  // for (auto& [viewport, image] : this->user_images_)
  // {
  //   ShowImage(this, viewport, image.get());
  // }

  // // finalize
  // this->platform_ui->SwapBuffers();
}


void MujocoWithROS2Render::RenderLoop()
{
  // Set timer callback (milliseconds)
  mjcb_time = Timer;

  // init abstract visualization
  mjv_defaultCamera(&this->cam);
  mjv_defaultOption(&this->opt);
  // InitializeProfiler(this);
  // InitializeSensor(this);

  // make empty scene
  // if (!is_passive_)
  // {
    mjv_defaultScene(&this->scn);
    mjv_makeScene(nullptr, &this->scn, kMaxGeom);
  // }

  if (!this->platform_ui->IsGPUAccelerated())
  {
    this->scn.flags[mjRND_SHADOW]     = 0;
    this->scn.flags[mjRND_REFLECTION] = 0;
  }

  // select default font
  int fontscale = ComputeFontScale(*this->platform_ui);
  this->font    = fontscale / 50 - 1;

  // make empty context
  this->platform_ui->RefreshMjrContext(nullptr, fontscale);

  // init state and uis
  std::memset(&this->uistate, 0, sizeof(mjuiState));
  std::memset(&this->ui0, 0, sizeof(mjUI));
  std::memset(&this->ui1, 0, sizeof(mjUI));

  auto [buf_width, buf_height] = this->platform_ui->GetFramebufferSize();
  this->uistate.nrect          = 1;
  this->uistate.rect[0].width  = buf_width;
  this->uistate.rect[0].height = buf_height;

  this->ui0.spacing   = mjui_themeSpacing(this->spacing);
  this->ui0.color     = mjui_themeColor(this->color);
  // this->ui0.predicate = UiPredicate;
  this->ui0.rectid    = 1;
  this->ui0.auxid     = 0;

  this->ui1.spacing   = mjui_themeSpacing(this->spacing);
  this->ui1.color     = mjui_themeColor(this->color);
  // this->ui1.predicate = UiPredicate;
  this->ui1.rectid    = 2;
  this->ui1.auxid     = 1;

  // set GUI adapter callbacks
  this->uistate.userdata = this;
  this->platform_ui->SetEventCallback(UiEvent);
  this->platform_ui->SetLayoutCallback(UiLayout);

  // populate uis with standard sections, open some sections initially
  this->ui0.userdata = this;
  this->ui1.userdata = this;
  mjui_add(&this->ui0, defFile);
  mjui_add(&this->ui0, this->def_option);
  mjui_add(&this->ui0, this->def_simulation);
  this->ui0.sect[0].state = 1;
  this->ui0.sect[1].state = 1;
  this->ui0.sect[2].state = 1;
  mjui_add(&this->ui0, this->def_watch);
  UiModify(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
  UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());

  // set VSync to initial value
  this->platform_ui->SetVSync(this->vsync);

  frames_          = 0;
  last_fps_update_ = mujoco_with_ros2::MujocoWithROS2Render::Clock::now();

  // run event loop
  while (!this->platform_ui->ShouldCloseWindow() && !this->exitrequest.load())
  {
    {
      const MutexLock lock(this->mtx);

      // // load model (not on first pass, to show "loading" label)
      // if (this->loadrequest == 1)
      // {
      //   this->LoadOnRenderThread(); // THis is needed
      // }
      // else if (this->loadrequest == 2)
      // {
      //   this->loadrequest = 1;
      // }

      // poll and handle events
      this->platform_ui->PollEvents();

      // upload assets if requested
      bool upload_notify = false;
      if (hfield_upload_ != -1)
      {
        mjr_uploadHField(m_, &platform_ui->mjr_context(), hfield_upload_);
        hfield_upload_ = -1;
        upload_notify  = true;
      }
      if (mesh_upload_ != -1)
      {
        mjr_uploadMesh(m_, &platform_ui->mjr_context(), mesh_upload_);
        mesh_upload_  = -1;
        upload_notify = true;
      }
      if (texture_upload_ != -1)
      {
        mjr_uploadTexture(m_, &platform_ui->mjr_context(), texture_upload_);
        texture_upload_ = -1;
        upload_notify   = true;
      }
      if (upload_notify)
      {
        cond_upload_.notify_all();
      }

      // update scene, doing a full sync if in fully managed mode
      // if (!is_passive_)
      // {
        Sync();
      // }
      // else if (m_passive_ && d_passive_)
      // {
      //   // the user has called Sync() in their code
      //   mjv_updateScene(
      //     m_passive_, d_passive_, &this->opt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);

      //   // add user geoms to scene
      //   int nusergeom = user_scn_geoms_.size();
      //   int ngeom     = std::min(nusergeom, this->scn.maxgeom - this->scn.ngeom);
      //   if (ngeom < nusergeom)
      //   {
      //     mj_warning(d_passive_, mjWARN_VGEOMFULL, this->scn.maxgeom);
      //   }
      //   std::memcpy(
      //     this->scn.geoms + this->scn.ngeom, user_scn_geoms_.data(), ngeom * sizeof(mjvGeom));
      //   this->scn.ngeom += ngeom;
      // }
    } // MutexLock (unblocks simulation thread)

    // render while simulation is running
    this->Render();

    // update FPS stat, at most 5 times per second
    auto now        = mujoco_with_ros2::MujocoWithROS2Render::Clock::now();
    double interval = Seconds(now - last_fps_update_).count();
    ++frames_;
    if (interval > 0.2)
    {
      last_fps_update_ = now;
      fps_             = frames_ / interval;
      frames_          = 0;
    }
  }

  const MutexLock lock(this->mtx);
  mjv_freeScene(&this->scn);
  // if (is_passive_)
  // {
  //   mj_deleteData(d_passive_);
  //   mj_deleteModel(m_passive_);
  // }

  this->exitrequest.store(2);
}

// add state to history buffer
// void MujocoWithROS2Render::AddToHistory()
// {
//   if (history_.empty())
//   {
//     return;
//   }

//   // circular increment of cursor
//   history_cursor_ = (history_cursor_ + 1) % nhistory_;

//   // add state at cursor
//   mjtNum* state = &history_[state_size_ * history_cursor_];
//   mj_getState(m_, d_, state, mjSTATE_INTEGRATION);
// }

// // inject Brownian noise
// void MujocoWithROS2Render::InjectNoise(int key)
// {
//   // no noise, return
//   if (ctrl_noise_std <= 0)
//   {
//     return;
//   }

//   // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
//   mjtNum rate  = mju_exp(-m_->opt.timestep / ctrl_noise_rate);
//   mjtNum scale = ctrl_noise_std * mju_sqrt(1 - rate * rate);

//   for (int i = 0; i < m_->nu; i++)
//   {
//     mjtNum bottom = 0, top = 0, midpoint = 0, halfrange = 1;
//     if (m_->actuator_ctrllimited[i])
//     {
//       bottom    = m_->actuator_ctrlrange[2 * i];
//       top       = m_->actuator_ctrlrange[2 * i + 1];
//       midpoint  = 0.5 * (top + bottom); // target of exponential decay
//       halfrange = 0.5 * (top - bottom); // scales noise
//     }

//     // overwrite midpoint with keyframe, if given
//     if (key >= 0)
//     {
//       midpoint = m_->key_ctrl[key * m_->nu + i];
//     }

//     // exponential convergence to midpoint at ctrl_noise_rate
//     d_->ctrl[i] = rate * d_->ctrl[i] + (1 - rate) * midpoint;

//     // add noise
//     d_->ctrl[i] += scale * halfrange * mju_standardNormal(nullptr);

//     // clip to range if limited
//     if (m_->actuator_ctrllimited[i])
//     {
//       d_->ctrl[i] = mju_clip(d_->ctrl[i], bottom, top);
//     }
//   }
// }

void MujocoWithROS2Render::UpdateHField(int hfieldid)
{
  MutexLock lock(this->mtx);
  if (!m_ || hfieldid < 0 || hfieldid >= m_->nhfield)
  {
    return;
  }
  hfield_upload_ = hfieldid;
  cond_upload_.wait(lock, [this]() { return hfield_upload_ == -1; });
}

void MujocoWithROS2Render::UpdateMesh(int meshid)
{
  MutexLock lock(this->mtx);
  if (!m_ || meshid < 0 || meshid >= m_->nmesh)
  {
    return;
  }
  mesh_upload_ = meshid;
  cond_upload_.wait(lock, [this]() { return mesh_upload_ == -1; });
}

void MujocoWithROS2Render::UpdateTexture(int texid)
{
  MutexLock lock(this->mtx);
  if (!m_ || texid < 0 || texid >= m_->ntex)
  {
    return;
  }
  texture_upload_ = texid;
  cond_upload_.wait(lock, [this]() { return texture_upload_ == -1; });
}

} // namespace mujoco_with_ros2
