import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description() -> LaunchDescription:
    project_root = os.environ.get("PROJECT_ROOT", "/root/project")
    default_world = os.path.join(project_root, "assets", "worlds", "tb3_world.sdf")

    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    bridge_config = LaunchConfiguration("bridge_config")

    bridge_config_default = PathJoinSubstitution(
        [FindPackageShare("tb3_nav2_slam_bringup"), "config", "bridge.yaml"]
    )

    gz_args = PythonExpression(["'-r -v4 ' + '", world, "'"])
    headless_effective = PythonExpression(
        ["'", headless, "' == 'True' or '", gui, "' == 'False'"]
    )

    urdf_file = PythonExpression(["'turtlebot3_' + '", model, "' + '.urdf.xacro'"])
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_description"), "urdf", urdf_file]
    )
    robot_description = Command([FindExecutable(name="xacro"), " ", urdf_path])

    model_dir = PythonExpression(["'turtlebot3_' + '", model, "'"])
    model_path = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_gazebo"), "models", model_dir, "model.sdf"]
    )

    tb3_models = PathJoinSubstitution([FindPackageShare("turtlebot3_gazebo"), "models"])
    assets_models = os.path.join(project_root, "assets", "models")
    existing_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_resources:
        resource_path = [
            TextSubstitution(text=f"{assets_models}:{existing_resources}:"),
            tb3_models,
        ]
    else:
        resource_path = [TextSubstitution(text=f"{assets_models}:"), tb3_models]

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("model", default_value="waffle"),
            DeclareLaunchArgument("robot_name", default_value="turtlebot3"),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("gui", default_value="True"),
            DeclareLaunchArgument("headless", default_value="False"),
            DeclareLaunchArgument("bridge_config", default_value=bridge_config_default),
            SetEnvironmentVariable("TURTLEBOT3_MODEL", model),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "gz_args": gz_args,
                    "headless": headless_effective,
                }.items(),
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", robot_name, "-file", model_path, "-z", "0.01"],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[{"config_file": bridge_config, "use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
