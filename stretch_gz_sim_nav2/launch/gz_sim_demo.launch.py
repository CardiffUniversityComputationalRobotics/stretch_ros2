import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stretch_sim_path = get_package_share_directory('stretch_gz_sim')
    stretch_navigation_path = get_package_share_directory('stretch_gz_sim_nav2')
    navigation_bringup_path = get_package_share_directory('nav2_bringup')
    map_path = get_package_share_directory('cucr_worlds_museum')
    
    use_sim_time_param = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    autostart_param = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Whether to autostart lifecycle nodes on launch')

    map_path_param = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(map_path,
                                   'maps', 'map.yaml'),
        description='Full path to the map.yaml file to use for navigation')

    world_gz_sim = os.path.join(map_path, 'worlds', 'default.sdf')

    stretch_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_sim_path, '/launch/bringup.launch.py']),
        launch_arguments={'world': world_gz_sim}.items())

    navigation_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/nav2_bringup_launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'), 
                          'autostart': LaunchConfiguration('autostart'),
                          'params_file': os.path.join(stretch_navigation_path, 'config', 'nav2_params.yaml'),
                          'map': LaunchConfiguration('map')}.items())

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_bringup_path, '/launch/rviz_launch.py']))    


    return LaunchDescription([
        use_sim_time_param,
        autostart_param,
        map_path_param,
        stretch_gz_sim,
        navigation_bringup_launch,
        rviz_launch,
    ])