cmake_minimum_required(VERSION 3.8)
project(mujoco_with_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

find_package(GLEW REQUIRED)

if(NOT DEFINED MUJOCO_DIR)
        set(MUJOCO_DIR "/usr/local") # CI setting
        message(WARNING "MUJOCO_DIR not specified. Using default ${MUJOCO_DIR}")
endif()
set(MUJOCO_INCLUDE_DIRS ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARIES
        ${MUJOCO_DIR}/lib/libmujoco.so
        )

add_executable(${PROJECT_NAME}_load_objects
        src/mujoco_initialize_and_load_objects.cpp
        )
target_include_directories(${PROJECT_NAME}_load_objects PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${MUJOCO_INCLUDE_DIRS}
        ${GLFW_INCLUDE_DIRS}
        )
ament_target_dependencies(${PROJECT_NAME}_load_objects
        rclcpp
        )
target_link_libraries(${PROJECT_NAME}_load_objects
        ${MUJOCO_LIBRARIES}
        ${GLFW_LIBRARIES}
        GLEW::GLEW
        )

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
  ${PROJECT_NAME}_load_objects
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
