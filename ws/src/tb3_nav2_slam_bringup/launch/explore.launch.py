from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    start_delay = LaunchConfiguration("start_delay")

    default_params = PathJoinSubstitution(
        [FindPackageShare("tb3_nav2_slam_bringup"), "config", "explore_params.yaml"]
    )

    remappings = [("tf", "/tf"), ("tf_static", "/tf_static")]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("start_delay", default_value="10.0"),
            TimerAction(
                period=start_delay,
                actions=[
                    Node(
                        package="explore_lite",
                        executable="explore",
                        name="explore_node",
                        namespace=namespace,
                        parameters=[params_file, {"use_sim_time": use_sim_time}],
                        remappings=remappings,
                        output="screen",
                    )
                ],
            ),
        ]
    )
