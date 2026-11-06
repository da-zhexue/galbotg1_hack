"""Launch the real-robot bridge or the RViz joint-state backend."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = "ctrl"
DESCRIPTION_PACKAGE = "galbot_one_golf_description"


def _is_equal(value: str, expected: str) -> IfCondition:
    return IfCondition(
        PythonExpression([f"'{value}' == '{expected}'"])
    )


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    use_rviz = LaunchConfiguration("rviz").perform(context).lower()
    if mode not in ("sim", "real"):
        raise RuntimeError(
            f"mode must be 'sim' or 'real', received: {mode}"
        )
    allow_motion = (
        LaunchConfiguration("allow_motion").perform(context).lower()
        in ("true", "1", "yes")
    )
    connect_sdk = (
        LaunchConfiguration("connect_sdk").perform(context).lower()
        in ("true", "1", "yes")
    )
    compare_states = (
        LaunchConfiguration("compare_states").perform(context).lower()
        in ("true", "1", "yes")
    )
    description_share = FindPackageShare(
        DESCRIPTION_PACKAGE
    ).perform(context)
    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    if not urdf_path:
        urdf_path = f"{description_share}/urdf/galbot_one_golf.urdf"

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()
    robot_description = robot_description.replace(
        'filename="meshes/',
        f'filename="package://{DESCRIPTION_PACKAGE}/meshes/',
    )

    real_condition = _is_equal(mode, "real")
    rviz_requested = IfCondition(
        PythonExpression(
            [
                f"'{use_rviz}' in ('true', '1', 'yes')",
            ]
        )
    )
    visual_condition = IfCondition(
        PythonExpression(
            [
                f"'{mode}' == 'sim' or ",
                f"'{use_rviz}' in ('true', '1', 'yes')",
                f" or '{str(compare_states).lower()}' == 'true'",
            ]
        )
    )
    compare_condition = IfCondition(
        PythonExpression(
            [f"'{str(compare_states).lower()}' == 'true'"]
        )
    )

    return [
        Node(
            package=PACKAGE_NAME,
            executable="rviz_joint_bridge",
            name="rviz_joint_bridge",
            condition=visual_condition,
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            condition=visual_condition,
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=rviz_requested,
            arguments=[
                "-d",
                f"{description_share}/config/rviz/model.rviz",
            ],
            output="screen",
        ),
        Node(
            package=PACKAGE_NAME,
            executable="real_robot_bridge",
            name="real_robot_bridge",
            condition=real_condition,
            parameters=[{
                "allow_motion": allow_motion,
                "connect_sdk": connect_sdk,
            }],
            output="screen",
        ),
        Node(
            package=PACKAGE_NAME,
            executable="state_comparator",
            name="state_comparator",
            condition=compare_condition,
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="sim",
            description="Backend: sim (RViz joint_states) or real (SDK).",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz in simulation mode.",
        ),
        DeclareLaunchArgument(
            "allow_motion",
            default_value="false",
            description="Allow the real backend to send robot commands.",
        ),
        DeclareLaunchArgument(
            "connect_sdk",
            default_value="true",
            description="Connect the real backend to galbot_sdk.",
        ),
        DeclareLaunchArgument(
            "compare_states",
            default_value="false",
            description="Compare real and simulated arm states.",
        ),
        DeclareLaunchArgument(
            "urdf_path",
            default_value="",
            description="URDF file used for RViz.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
