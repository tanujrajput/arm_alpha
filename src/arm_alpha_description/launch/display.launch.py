# Standard library
import os

# ROS 2 utility to get the path of installed packages
from ament_index_python.packages import get_package_share_directory

# Launch system imports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

# ROS 2 launch actions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the path to the 'arduinobot_description' package
    # This is where the URDF/Xacro, meshes, and RViz configs are stored
    arduinobot_description_dir = get_package_share_directory("arm_alpha_description")

    # Declare a launch argument called "model"
    # Default: path to the URDF Xacro file of the robot
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            arduinobot_description_dir, "urdf", "arm_alpha.urdf.xacro"
        ),
        description="Absolute path to robot urdf file"
    )

    # Process the Xacro file into a URDF string and store it in 'robot_description'
    # This is what robot_state_publisher expects
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Node 1: Robot State Publisher
    # Publishes TF transforms of all robot links based on the URDF description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Node 2: Joint State Publisher GUI
    # Provides a simple GUI with sliders to move each joint of the robot manually
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # Node 3: RViz2
    # Launch RViz with a predefined configuration file to visualize the robot model
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(arduinobot_description_dir, "rviz", "display.rviz")],
    )

    # Return the complete launch description with all declared actions/nodes
    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
