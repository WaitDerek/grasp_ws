from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_daemon = LaunchConfiguration("start_daemon")
    detector_mode = LaunchConfiguration("detector_mode")
    graspness_dir = LaunchConfiguration("graspness_dir")
    checkpoint_path = LaunchConfiguration("checkpoint_path")
    runtime_dir = LaunchConfiguration("runtime_dir")
    config_file = LaunchConfiguration("config_file")
    scene_topic = LaunchConfiguration("scene_topic")
    target_topic = LaunchConfiguration("target_topic")
    visualize = LaunchConfiguration("visualize")
    visualization_grasps = LaunchConfiguration("visualization_grasps")

    return LaunchDescription(
        [
            SetEnvironmentVariable("OMP_NUM_THREADS", "12"),
            DeclareLaunchArgument("start_daemon", default_value="true"),
            DeclareLaunchArgument("detector_mode", default_value="basic"),
            DeclareLaunchArgument(
                "graspness_dir",
                default_value=EnvironmentVariable(
                    "GRASPNESS_C_DIR",
                    default_value=PathJoinSubstitution(
                        [
                            FindPackagePrefix("grasp_orchestrator"),
                            "..",
                            "src",
                            "graspness_c",
                        ]
                    ),
                ),
            ),
            DeclareLaunchArgument(
                "checkpoint_path",
                default_value=EnvironmentVariable(
                    "GRASPNESS_CHECKPOINT",
                    default_value=PathJoinSubstitution(
                        [
                            graspness_dir,
                            "logs",
                            "log_kn",
                            "minkuresunet_realsense.tar",
                        ]
                    ),
                ),
            ),
            DeclareLaunchArgument(
                "runtime_dir",
                default_value=EnvironmentVariable(
                    "GRASP_RUNTIME_DIR",
                    default_value=PathJoinSubstitution(
                        [
                            FindPackagePrefix("grasp_orchestrator"),
                            "..",
                            "runtime",
                            "graspness",
                        ]
                    ),
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("grasp_orchestrator"),
                        "config",
                        "camera_topics_r1pro.yaml",
                    ]
                ),
                description="ROS parameter file containing the RGB-D camera topics.",
            ),
            DeclareLaunchArgument(
                "scene_topic",
                default_value="/perception/task1/rest_point_cloud",
            ),
            DeclareLaunchArgument(
                "target_topic",
                default_value="/perception/task1/target_point_cloud",
            ),
            DeclareLaunchArgument("visualize", default_value="false"),
            DeclareLaunchArgument("visualization_grasps", default_value="10"),
            ExecuteProcess(
                cmd=[
                    FindExecutable(name="python3"),
                    PathJoinSubstitution([graspness_dir, "grasp_daemon.py"]),
                    "--checkpoint_path",
                    checkpoint_path,
                    "--save_dir",
                    runtime_dir,
                    "--visualize",
                    visualize,
                    "--visualization_grasps",
                    visualization_grasps,
                    "--ros-args",
                    "--params-file",
                    config_file,
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_daemon,
                            "'.lower() == 'true' and '",
                            detector_mode,
                            "' == 'basic'",
                        ]
                    )
                ),
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    FindExecutable(name="python3"),
                    PathJoinSubstitution([graspness_dir, "infer_atec.py"]),
                    "--checkpoint_path",
                    checkpoint_path,
                    "--save_dir",
                    runtime_dir,
                    "--scene_topic",
                    scene_topic,
                    "--target_topic",
                    target_topic,
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_daemon,
                            "'.lower() == 'true' and '",
                            detector_mode,
                            "' == 'atec'",
                        ]
                    )
                ),
                output="screen",
            ),
            Node(
                package="grasp_orchestrator",
                executable="detection_bridge_service",
                name="detection_bridge_service",
                output="screen",
                prefix=[FindExecutable(name="python3")],
                parameters=[
                    config_file,
                    {
                        "save_dir": runtime_dir,
                        "input_mode": detector_mode,
                        "scene_topic": scene_topic,
                        "target_topic": target_topic,
                    }
                ],
            ),
        ]
    )
