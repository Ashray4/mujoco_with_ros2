#include <vector>
#include <string>

#include <mujoco_with_ros2/mujoco_initialize_and_load_objects.h>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

namespace mujoco_with_ros2 {
class ManageMujoco
{
private:
    //
    int n_joints_;
    std::vector<std::string> mujoco_joint_names_;
    std::vector<int> mujoco_joint_ids_;

    //Mujoco Simulation Variables
    mjModel* mujoco_model_;
    mjData* mujoco_data_;
    MujocoInitLoadObjects& load_mujoco_object_simulation_;

    ManageMujoco(int n_joints,std::vector<std::string>& mujoco_joint_names);
    ~ManageMujoco();
public:
    int totalJoints();
    const std::vector<std::string>& getJointNames();
    
};
}
