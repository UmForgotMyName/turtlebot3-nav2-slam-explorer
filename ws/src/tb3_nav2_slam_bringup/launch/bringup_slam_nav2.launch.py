from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_world = PathJoinSubstitution(
        [FindPackageShare("tb3_nav2_slam_bringup"), "assets", "worlds", "tb3_world.sdf"]
    )
    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    slam = LaunchConfiguration("slam")
    rviz = LaunchConfiguration("rviz")
    rviz_software = LaunchConfiguration("rviz_software")
    explore = LaunchConfiguration("explore")
    explore_params_file = LaunchConfiguration("explore_params_file")
    odom_tf = LaunchConfiguration("odom_tf")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bridge_config = LaunchConfiguration("bridge_config")
    headless_effective = PythonExpression(
        ["'", headless, "' == 'True' or '", gui, "' == 'False'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("model", default_value="waffle"),
            DeclareLaunchArgument("gui", default_value="True"),
            DeclareLaunchArgument("headless", default_value="False"),
            DeclareLaunchArgument("slam", default_value="True"),
            DeclareLaunchArgument("rviz", default_value="True"),
            DeclareLaunchArgument("rviz_software", default_value="True"),
            DeclareLaunchArgument("explore", default_value="False"),
            DeclareLaunchArgument(
                "explore_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tb3_nav2_slam_bringup"), "config", "explore_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("odom_tf", default_value="True"),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument(
                "bridge_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tb3_nav2_slam_bringup"), "config", "bridge.yaml"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("tb3_nav2_slam_bringup"), "launch", "sim_gz.launch.py"]
                    )
                ),
                launch_arguments={
                    "world": world,
                    "model": model,
                    "use_sim_time": use_sim_time,
                    "gui": gui,
                    "headless": headless_effective,
                    "bridge_config": bridge_config,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("tb3_nav2_slam_bringup"), "launch", "slam.launch.py"]
                    )
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("tb3_nav2_slam_bringup"), "launch", "nav2.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam": slam,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("tb3_nav2_slam_bringup"), "launch", "rviz.launch.py"]
                    )
                ),
                condition=IfCondition(rviz),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "rviz_software": rviz_software,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("tb3_nav2_slam_bringup"), "launch", "explore.launch.py"]
                    )
                ),
                condition=IfCondition(explore),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": explore_params_file,
                }.items(),
            ),
            Node(
                package="tb3_nav2_slam_bringup",
                executable="odom_to_tf",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(odom_tf),
            ),
        ]
    )
