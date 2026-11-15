"""Start application nodes. MAVROS and the calibrated camera driver are external."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base = PathJoinSubstitution([FindPackageShare("start"), "config"])
    args = [
        DeclareLaunchArgument(k, default_value=v)
        for k, v in dict(
            use_sim_time="false",
            parameter_write_enabled="false",
            upload_enabled="false",
            handoff_enabled="false",
            live_output="false",
            partial_attitude_verified="false",
            parameter_config=PathJoinSubstitution([base, "parameter_manager.yaml"]),
            fc_profile=PathJoinSubstitution([base, "fc_parameters.yaml"]),
            mission_config=PathJoinSubstitution([base, "mission.yaml"]),
            perception_config=PathJoinSubstitution([base, "perception.yaml"]),
            control_config=PathJoinSubstitution([base, "control.yaml"]),
        ).items()
    ]
    sim = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}

    def flag(name):
        return {name: ParameterValue(LaunchConfiguration(name), value_type=bool)}

    def node(package, name, configs, overrides=None):
        return Node(
            package=package,
            executable=name,
            name=name,
            output="screen",
            parameters=[*[LaunchConfiguration(c) for c in configs], sim, overrides or {}],
        )

    return LaunchDescription(
        args
        + [
            node("perception", "net_det", ["perception_config"]),
            node("perception", "net_pos", ["perception_config"]),
            node(
                "autoland",
                "parameter_manager",
                ["parameter_config"],
                {
                    **flag("parameter_write_enabled"),
                    "profile_path": ParameterValue(
                        LaunchConfiguration("fc_profile"), value_type=str
                    ),
                },
            ),
            node("autoland", "mission_manager", ["mission_config"], flag("upload_enabled")),
            node("autoland", "landing_supervisor", ["control_config"], flag("handoff_enabled")),
            node("autoland", "vision_controller", ["control_config"]),
            node(
                "autoland",
                "fcu_interface",
                ["control_config"],
                {**flag("live_output"), **flag("partial_attitude_verified")},
            ),
        ]
    )
