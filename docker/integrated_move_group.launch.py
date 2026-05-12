#!/usr/bin/env -S ros2 launch
"""Minimal PhantomX MoveIt launch for the integrated OpenCV -> IK dry run.

The upstream launch file targets a full controller/RViz setup.  For the Docker
integration test we only need robot_description, robot_description_semantic,
kinematics and move_group's /compute_ik service.
"""

from os import path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    with open(path.join(package_path, relative_path), "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def flatten(prefix, value):
    flat = {}
    if isinstance(value, dict):
        for key, child in value.items():
            flat.update(flatten(f"{prefix}.{key}" if prefix else key, child))
    else:
        flat[prefix] = value
    return flat


def generate_launch_description():
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    moveit_config_package = "phantomx_pincher_moveit_config"
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    collision = LaunchConfiguration("collision")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([FindPackageShare(description_package), description_filepath]),
                " ",
                "name:=",
                name,
                " ",
                "prefix:=",
                prefix,
                " ",
                "collision:=",
                collision,
                " ",
                "ros2_control:=false ",
                "ros2_control_plugin:=fake ",
                "ros2_control_command_interface:=position ",
                "mimic_finger_joints:=false ",
                "gazebo_preserve_fixed_joint:=false",
            ]
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(moveit_config_package),
                        "srdf",
                        "phantomx_pincher.srdf.xacro",
                    ]
                ),
                " ",
                "name:=",
                name,
                " ",
                "prefix:=",
                prefix,
                " ",
                "use_real_gripper:=false",
            ]
        )
    }

    kinematics_yaml = load_yaml(moveit_config_package, path.join("config", "kinematics.yaml"))
    # The Jazzy image ships KDL by default; the upstream config's LMA plugin is not
    # present in the binary packages used by this integrated test image.
    kinematics_yaml["arm"]["kinematics_solver"] = "kdl_kinematics_plugin/KDLKinematicsPlugin"
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            moveit_config_package, path.join("config", "joint_limits.yaml")
        )
    }

    ompl_yaml = load_yaml(moveit_config_package, path.join("config", "ompl_planning.yaml"))
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl.planning_plugins": ["ompl_interface/OMPLPlanner"],
        "ompl.request_adapters": [
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/ValidateWorkspaceBounds",
            "default_planning_request_adapters/CheckStartStateBounds",
            "default_planning_request_adapters/CheckStartStateCollision",
        ],
        "ompl.response_adapters": [
            "default_planning_response_adapters/AddTimeOptimalParameterization",
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath",
        ],
        "ompl.start_state_max_bounds_error": 0.1,
        **flatten("ompl", ompl_yaml),
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    trajectory_execution = {
        "allow_trajectory_execution": False,
        "moveit_manage_controllers": False,
    }

    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="phantomx_pincher_description",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=path.join("urdf", "phantomx_pincher.urdf.xacro"),
        ),
        DeclareLaunchArgument("name", default_value="phantomx_pincher"),
        DeclareLaunchArgument("prefix", default_value="phantomx_pincher_"),
        DeclareLaunchArgument("collision", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                {"publish_frequency": 50.0, "use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                robot_description_planning,
                planning_pipeline,
                trajectory_execution,
                planning_scene_monitor_parameters,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)
