cmake_minimum_required(VERSION 3.8)
project(mujoco_with_ros2)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools Boost control_msgs
)
foreach(dep IN LISTS THIS_PACKAGE_INCLUDE_DEPENDS)
  find_package(${dep} REQUIRED)
endforeach()

# --- 3rd-party ---
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
find_package(GLEW REQUIRED)

# MuJoCo (keep your fallback if no CMake package)
if(NOT DEFINED MUJOCO_DIR)
  set(MUJOCO_DIR "/usr/local")
  message(WARNING "MUJOCO_DIR not specified. Using default ${MUJOCO_DIR}")
endif()
set(MUJOCO_INCLUDE_DIRS ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARIES    ${MUJOCO_DIR}/lib/libmujoco.so)

# ===================================================================
# 1) Common library (code shared by plugin + node)  <-- NEW
#    Move the *non-main* parts from mujoco_initialize_and_load_objects.cpp here
# ===================================================================
# add_library(${PROJECT_NAME}_core
#   src/mujoco_initialize_and_load_objects.cpp  # <- no main() in this file
# )
# target_include_directories(${PROJECT_NAME}_core PUBLIC
#   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${MUJOCO_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS}
# )
# target_link_libraries(${PROJECT_NAME}_core PUBLIC
#   ${MUJOCO_LIBRARIES} ${GLFW_LIBRARIES} GLEW::GLEW
# )
# ament_target_dependencies(${PROJECT_NAME}_core PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ===================================================================
# 2) Hardware-interface plugin library
# ===================================================================
add_library(${PROJECT_NAME} SHARED
src/mujoco_initialize_and_load_objects.cpp
src/manage_mujoco.cpp  
src/mujoco_ros2_system_control.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mujoco_with_ros2>
  ${MUJOCO_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(${PROJECT_NAME} PUBLIC 
  ${MUJOCO_LIBRARIES} ${GLFW_LIBRARIES} GLEW::GLEW
)

target_compile_definitions(${PROJECT_NAME} PRIVATE MUJOCO_WITH_ROS2_SYSTEM_BUILDING_DLL)

pluginlib_export_plugin_description_file(hardware_interface mujoco_with_ros2_system.xml)

add_executable(${PROJECT_NAME}_simulator_test
src/robot_mujoco_simulator.cpp)

target_include_directories(${PROJECT_NAME}_simulator_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mujoco_with_ros2>
    ${MUJOCO_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} mujoco_with_ros2
)
target_link_libraries(${PROJECT_NAME}_simulator_test PUBLIC
    ${PROJECT_NAME}  # This links to your main library
    ${MUJOCO_LIBRARIES} 
    ${GLFW_LIBRARIES} 
    GLEW::GLEW
)

ament_target_dependencies(${PROJECT_NAME}_simulator_test PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})


# ===================================================================
# Install & export
# ===================================================================
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY models mujoco_with_ros2_bringup/launch config
        DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
${PROJECT_NAME}_simulator_test
DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)

ament_package()
