import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    # Get package paths
    mujoco_ros2_ur_path = get_package_share_directory("mujoco_with_ros2")

    # Load and process xacro file
    xacro_file = os.path.join(
        mujoco_ros2_ur_path, "models", "ur5e","urdf_ros2", "ur5e_mujoco.urdf.xacro"
    )
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {"robot_description": doc.toxml()}

    # Path to controller config
    controller_config_file = os.path.join(
        mujoco_ros2_ur_path, "config", "ur5e_controllers_mujoco.yaml"
    )
    # griiper_controller_config_file = os.path.join(mujoco_ros2_ur_path, 'config', 'robotiq_controller_mujoco.yaml')
    mujoco_model_path = os.path.join(
        mujoco_ros2_ur_path, "models", "ur5e","urdf","scene.xml"
    )

    rviz_config_file = os.path.join(mujoco_ros2_ur_path,
                                          'launch',
                                          'camera_demo.rviz')

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_file,robot_description],
        output="both",
    )

    # Robot state publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     parameters=[{"use_sim_time": True}],
    #     arguments=["-d", rviz_config_file],
    # )


    controllers_active = [
        "joint_state_broadcaster",
        "interpolation_controller",
        "force_torque_sensor_broadcaster",
    ]
    
    controllers_inactive = [
        "adaptive_mpc_controller",
        "interpoaltion_example_controller",
        "effort_controller",
        "joint_trajectory_controller"
    ]
    
    spawner_nodes = []
    
    # Spawn and activate these controllers
    for controller in controllers_active:
        spawner_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
                output="screen",
            )
        )
    
    # Spawn but keep inactive (loaded but not started)
    for controller in controllers_inactive:
        spawner_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager", "--inactive"],
                output="screen",
            )
        )
    
    # gripper_controllers = [
    #     "gripper_controller",

    #     "gripper_state_controller"
    # ]
    # gripper_spawner_nodes = [
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=[gripper_controllers, "-c", "/robotiq/controller_manager"],
    #         output="screen"
    #     ) for gripper_controllers in gripper_controllers
    # ]
    return LaunchDescription(
        [
            control_node,
            node_robot_state_publisher,
            # rviz_node,
            *spawner_nodes,

            # *gripper_spawner_nodes
        ]
    )
