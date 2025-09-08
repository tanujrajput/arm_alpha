import os  # For handling file paths
from launch import LaunchDescription  # Main class used to describe the launch
from launch.actions import DeclareLaunchArgument  # To declare launch-time arguments
from launch.conditions import UnlessCondition  # Conditional execution of nodes
from launch.substitutions import LaunchConfiguration, Command  # Dynamic values and commands
from launch_ros.actions import Node  # Used to launch ROS 2 nodes
from launch_ros.parameter_descriptions import ParameterValue  # For dynamic node parameters
from ament_index_python.packages import get_package_share_directory  # To locate ROS 2 packages

# Function that generates the launch description
def generate_launch_description():

    # Declare a launch argument "is_sim" to indicate if we are running in simulation
    # Default is True, meaning simulation is running
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    # Get the value of the "is_sim" launch argument
    is_sim = LaunchConfiguration("is_sim")

    # Generate the robot_description parameter dynamically using xacro
    # Runs: xacro <path_to_arm_alpha.urdf.xacro>
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("arm_alpha_description"),
                    "urdf",
                    "arm_alpha.urdf.xacro",
                ),
            ]
        ),
        value_type=str,  # Ensure the output is interpreted as a string
    )

    # Node to publish the robot's state (TFs, joint transforms)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # Only launch this node if NOT in simulation (UnlessCondition)
        # In simulation, Gazebo often publishes TFs
        condition=UnlessCondition(is_sim),
        parameters=[{"robot_description": robot_description}],
    )

    # Launch the ros2_control node (controller manager)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",  # Core node that manages controllers
        parameters=[
            {"robot_description": robot_description, "use_sim_time": is_sim},  # Robot URDF + simulation time
            os.path.join(
                get_package_share_directory("arm_alpha_controller"),
                "config",
                "arm_alpha_controllers.yaml",  # YAML file defining all controllers
            ),
        ],
        condition=UnlessCondition(is_sim),  # Only launch if not in simulation
    )

    # Spawner node for the joint_state_broadcaster
    # This publishes joint positions to the /joint_states topic
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",  # Utility to start controllers in ros2_control
        arguments=[
            "joint_state_broadcaster",  # Controller to spawn
            "--controller-manager",
            "/controller_manager",  # Name of the controller manager node
        ],
    )

    # Spawner node for the main arm controller
    # Sends commands to joint_1, joint_2, joint_3 based on trajectories
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawner node for the gripper controller
    # Sends commands to the gripper joint (joint_4)
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # Return the LaunchDescription containing all declared arguments and nodes
    return LaunchDescription(
        [
            is_sim_arg,  # Launch argument to determine simulation mode
            robot_state_publisher_node,  # Publishes robot TFs
            controller_manager,  # Launches ros2_control
            joint_state_broadcaster_spawner,  # Spawns joint_state_broadcaster
            arm_controller_spawner,  # Spawns arm_controller
            gripper_controller_spawner,  # Spawns gripper_controller
        ]
    )
