from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnExecutionComplete

from os.path import join


def generate_launch_description():
    # This allows us to use the rviz_config variable in substitutions in this launch description.
    rviz_config = LaunchConfiguration('rviz_config', default="general.rviz")

    base_path = get_package_share_directory("roomba")
    # Include the gazebo setup
    gazebo = IncludeLaunchDescription(join(base_path, "launch","gazebo.launch.py"))


    # SLAM Toolbox for mapping
    slam_toolbox = Node( package='slam_toolbox', 
                         executable='async_slam_toolbox_node', 
                         parameters=[
                                get_package_share_directory('roomba') + '/config/mapping.yaml'
                        ], output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([base_path, 'config', rviz_config])
        ]
    )

    return LaunchDescription([gazebo, slam_toolbox, rviz]) 