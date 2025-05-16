#!/usr/bin/env python3
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Iris - Include this exactly the same way as in iris_runway.launch.py
    iris = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_gz_bringup"),
                        "launch",
                        "robots",
                        "iris.launch.py",
                    ]
                ),
            ]
        )
    )

    # Gazebo
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            f'{Path(pkg_project_gazebo) / "worlds" / "iris_aruco_world.sdf"}'
        }.items(),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )

    # RViz - Use the SAME rviz config as in iris_runway
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", f'{Path(pkg_project_bringup) / "rviz" / "iris.rviz"}'],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    
    # Bridge for ArUco target control
    aruco_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/model/target_aruco/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
        output="screen",
    )
    
    # Launch the circle motion script if enabled
    circle_motion = Node(
        package="ardupilot_gz_bringup",
        executable="circle_motion.py",
        condition=IfCondition(LaunchConfiguration("use_circle_motion")),
        output="screen",
    )

    # Topic tools relay for TF (copied from iris.launch.py)
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        arguments=[
            "/gz/tf",
            "/tf",
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            DeclareLaunchArgument(
                "use_circle_motion", default_value="false", 
                description="Enable automatic circle motion for ArUco target."
            ),
            DeclareLaunchArgument(
                "use_gz_tf", default_value="true", description="Use Gazebo TF."
            ),
            gz_sim_server,
            gz_sim_gui,
            iris,
            aruco_bridge,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=aruco_bridge,
                    on_start=[
                        topic_tools_tf
                    ]
                )
            ),
            rviz,
            circle_motion,
        ]
    )