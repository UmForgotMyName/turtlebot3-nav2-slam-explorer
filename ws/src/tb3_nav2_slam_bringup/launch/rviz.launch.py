from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_software = LaunchConfiguration("rviz_software")

    default_rviz = PathJoinSubstitution(
        [FindPackageShare("tb3_nav2_slam_bringup"), "config", "rviz_nav2_slam.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("rviz_software", default_value="True"),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"},
                condition=IfCondition(rviz_software),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=UnlessCondition(rviz_software),
                output="screen",
            ),
        ]
    )
