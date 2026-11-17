"""Run MAVROS router and UAS in separate processes on ROS 2 Humble.

The packaged mavros_node enables intra-process transport between executors.
Use the supported components over DDS to avoid stalls in that path.
"""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    directory = Path(__file__).resolve().parent
    share = Path("/opt/ros/humble/share/mavros/launch")
    router = ComposableNodeContainer(
        name="mavros_router_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="mavros",
                plugin="mavros::router::Router",
                name="mavros_router",
                parameters=[{
                    "fcu_urls": ["tcp://127.0.0.1:5762"],
                    "uas_urls": ["/uas1"],
                    "use_sim_time": True,
                }],
                extra_arguments=[{"use_intra_process_comms": False}],
            )
        ],
    )
    uas = ComposableNodeContainer(
        name="mavros_uas_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="mavros",
                plugin="mavros::uas::UAS",
                name="mavros",
                parameters=[
                    str(share / "apm_config.yaml"),
                    str(share / "apm_pluginlists.yaml"),
                    str(directory / "mavros.yaml"),
                    {"uas_url": "/uas1", "system_id": 1, "component_id": 191,
                     "target_system_id": 1, "target_component_id": 1,
                     "fcu_protocol": "v2.0"},
                ],
                extra_arguments=[{"use_intra_process_comms": False}],
            )
        ],
    )
    return LaunchDescription([router, uas])
