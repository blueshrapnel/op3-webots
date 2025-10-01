# robot_and_manager.launch.py
# launch both the robot description and the op3 manager

import os
import subprocess
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, OpaqueFunction, EmitEvent
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory


ROBOT_NAME_IN_WEBOTS = "ROBOTIS_OP3_0"  # must match the robot's name in field.wbt


def spawn_controller(context, webots_action, gain_file, robot_name):
    return [Node(
        package="op3_webots_ros2",
        executable="op3_extern_controller",
        name=f"controller_{robot_name.lower()}",
        output="screen",
        parameters=[{"gain_file_path": gain_file}],
        # bind the controller to this robot inside Webots
        additional_env={"WEBOTS_CONTROLLER_URL": f"tcp://localhost:1234/{robot_name}"},
     )]


def spawn_manager(context):
    pkg_share = get_package_share_directory("op3_webots_ros2")
    # Manager expects sim mode + OP3 config files
    return [Node(
        package="op3_manager",
        executable="op3_manager",
        name="op3_manager",
        output="screen",
        parameters=[{
            "simulation": True,
            "simulation_robot_name": "robotis_op3",
            "offset_file_path": os.path.join(pkg_share, "config", "offset.yaml"),
            "robot_file_path": os.path.join(pkg_share, "config", "OP3.robot"),
            "init_file_path": os.path.join(pkg_share, "config", "dxl_init_OP3.yaml"),
            "team_number": 0,
            "player_number": 1,
        }],
        # If you decide to isolate graphs later, match the controller's domain
        # additional_env={"ROS_DOMAIN_ID": "0"},

        # Make sure we never respawn and we kill hard if needed
        respawn=False,
        sigterm_timeout='2.0',   # seconds to wait after SIGTERM
        sigkill_timeout='2.0',   # then SIGKILL
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory("op3_webots_ros2")
    world = os.path.join(pkg_share, "worlds", "field.wbt")
    gain_file = os.path.join(pkg_share, "resource", "op3_pid_gain_default.yaml")

    webots = WebotsLauncher(world=world)

    # Start controller ~2s after Webots starts
    start_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=webots,
            on_start=[TimerAction(
                period=2.0,
                actions=[OpaqueFunction(
                    function=spawn_controller,
                    args=(webots, gain_file, ROBOT_NAME_IN_WEBOTS),
                )]
            )]
        )
    )

    # start manager a little later to let the controller advertise topics
    start_manager = RegisterEventHandler(
        OnProcessStart(
            target_action=webots,
            on_start=[TimerAction(
                period=3.5,
                actions=[OpaqueFunction(
                    function=spawn_manager,
                )]
            )]
        )
    )

    # clean shutdown when Webots exits
    shutdown = RegisterEventHandler(
        OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription([webots, start_controller, start_manager, shutdown])

