import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Binary path in the workspace (adjusts to user's home workspace)
    binary_path = os.path.join(os.path.expanduser('~'), 'CatamaranDependencies', 'SurroundViewRealTime', 'build', 'surround_view_ros_subscriber')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[binary_path],
            output='screen'
        )
    ])
