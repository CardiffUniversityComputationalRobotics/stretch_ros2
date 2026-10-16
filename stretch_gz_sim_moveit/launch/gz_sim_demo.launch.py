import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Main packages
    pkg_stretch_gz_sim = get_package_share_directory('stretch_gz_sim')
    pkg_stretch_gz_sim_moveit = get_package_share_directory('stretch_gz_sim_moveit')

    # Gazebo Sim (ros2 launch stretch_gz_sim bringup.launch.py)
    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_stretch_gz_sim, 'launch', 'bringup.launch.py')),
    )

    # Wait for Controllers
    # The gripper controller is the last one spawned by bringup.launch.py
    wait_for_controllers = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'until ros2 control list_controllers 2>/dev/null'
            ' | grep -q "stretch_gripper_controller.*active"; do sleep 2; done',
        ],
        output='screen',
    )

    # MoveIt (ros2 launch stretch_gz_sim_moveit move_group.launch.py)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_stretch_gz_sim_moveit, 'launch', 'move_group.launch.py')),
    )

    # Wait for MoveIt
    # move_group advertises /move_action once it is ready to plan
    wait_for_move_group = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'until ros2 action list 2>/dev/null | grep -qx /move_action; do sleep 2; done',
        ],
        output='screen',
    )

    # Sim Time (ros2 param set /move_group use_sim_time true)
    move_group_sim_time = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/move_group', 'use_sim_time', 'true'],
        output='screen',
    )

    # RViz (ros2 launch stretch_gz_sim_moveit moveit_rviz.launch.py)
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_stretch_gz_sim_moveit, 'launch', 'moveit_rviz.launch.py')),
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=wait_for_controllers,
                    on_exit=[move_group, wait_for_move_group],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=wait_for_move_group,
                    on_exit=[move_group_sim_time],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=move_group_sim_time,
                    on_exit=[moveit_rviz],
                )
            ),
            # Nodes and Launches
            gazebo_bringup,
            wait_for_controllers,
        ]
    )
