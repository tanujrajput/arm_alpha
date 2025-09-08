# Import standard Python modules
import os  # Used to interact with environment variables and file paths
from pathlib import Path  # Provides object-oriented paths handling

# Import ROS 2 Python launch tools
from ament_index_python.packages import get_package_share_directory  
# Used to locate the file paths of installed ROS 2 packages

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
# DeclareLaunchArgument -> allows launch-time arguments to be passed to the launch file
# IncludeLaunchDescription -> include another launch file inside this launch file
# SetEnvironmentVariable -> set environment variables for this launch

from launch.substitutions import Command, LaunchConfiguration
# Command -> run a command and use its output
# LaunchConfiguration -> get the value of a launch argument during runtime

from launch.launch_description_sources import PythonLaunchDescriptionSource
# Used to include another launch file written in Python

from launch_ros.actions import Node
# Used to define and launch ROS 2 nodes
from launch_ros.parameter_descriptions import ParameterValue
# Used to define parameter values that can be dynamically generated

# Function that defines and returns the launch description
def generate_launch_description():
    # Get the absolute path of the "arm_alpha_description" package
    arm_alpha_description = get_package_share_directory("arm_alpha_description")

    # Declare a launch argument "model" that specifies the path to the robot URDF/XACRO file
    # This allows the user to override the default robot model if needed
    model_arg = DeclareLaunchArgument(
        name="model",  # Argument name
        default_value=os.path.join(
            arm_alpha_description, "urdf", "arm_alpha.urdf.xacro"  # Default URDF/XACRO path
        ),
        description="Absolute path to robot urdf file"  # Help text shown to the user
    )

    # Set environment variable for Gazebo simulation resources
    # GZ_SIM_RESOURCE_PATH tells Gazebo where to find SDF models, meshes, and other simulation assets
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(arm_alpha_description).parent.resolve())  # Parent folder of package for resources
        ]
    )

    # Get the ROS distribution (e.g., "humble", "iron") from environment variables
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    # Define the robot_description parameter for the robot_state_publisher node
    # It dynamically generates the URDF using xacro and the launch argument "model"
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    # Define the robot_state_publisher node
    # This node publishes the robot's joint states and transforms based on the URDF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",  # ROS 2 package
        executable="robot_state_publisher",  # Executable in the package
        parameters=[{
            "robot_description": robot_description,  # URDF parameter
            "use_sim_time": True  # Use simulated time for Gazebo
        }]
    )

    # Include the Gazebo simulation launch file from the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            # Pass arguments to the included launch file
            ("gz_args", [" -v 4 -r empty.sdf "])  # Verbose level 4, start with empty world
        ]
    )

    # Node to spawn the robot in Gazebo using the robot_description topic
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",  # This executable spawns entities from a description topic
        output="screen",  # Show logs on the terminal
        arguments=[
            "-topic", "robot_description",  # Use the robot_description parameter/topic
            "-name", "arm_alpha"  # Name of the robot entity in Gazebo
        ],
    )

    # Node to bridge Gazebo simulation clock to ROS 2 /clock topic
    # Enables synchronization of ROS 2 nodes with Gazebo simulation time
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",  # Bridges topics between ROS 2 and Gazebo
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",  # Bridge Gazebo clock to ROS clock
        ]
    )

    # Return the launch description with all declared arguments, environment variables, and nodes
    return LaunchDescription([
        model_arg,  # The robot model argument
        gazebo_resource_path,  # Set Gazebo resource path
        robot_state_publisher_node,  # Publish robot state transforms
        gazebo,  # Launch Gazebo simulation
        gz_spawn_entity,  # Spawn robot in Gazebo
        gz_ros2_bridge  # Bridge simulation clock to ROS 2
    ])
