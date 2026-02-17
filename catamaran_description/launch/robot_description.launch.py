from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare 

def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare("catamaran_description"),
        "urdf",
        "catamaran.xacro"
    ])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": Command(["xacro ", xacro_file])
            }]
        )
    ])