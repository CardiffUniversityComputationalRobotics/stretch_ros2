import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Main package
    pkg_stretch_gz_sim = get_package_share_directory('stretch_gz_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    use_base_footprint = LaunchConfiguration("use_base_footprint")
    declare_use_base_footprint_cmd = DeclareLaunchArgument(
        "use_base_footprint",
        default_value= "true",
        description="add link base_footprint",
    )

    # Robot State Publisher
    robot_description_path = os.path.join(
        pkg_stretch_gz_sim,
        "urdf",
        "stretch_re1",
        "stretch_description_standard.xacro"
    )
    robot_description_config = xacro.process_file(
        robot_description_path
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Gazebo Sim
    world = LaunchConfiguration("world")
    use_gui = LaunchConfiguration("use_gui")

    world_path = os.path.join(pkg_stretch_gz_sim, 'worlds', 'empty_world.sdf')
    
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value= world_path,
        description="Path of the world to show.",
    )

    declare_use_gui_cmd = DeclareLaunchArgument(
        "use_gui",
        default_value= "true",
        description="use GUI of gz sim",
    )

    world_str_path_headless = [TextSubstitution(text='-s -r '), world]
    world_str_path = [TextSubstitution(text='-r '), world]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_str_path}.items(),
        condition=IfCondition(use_gui),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_str_path_headless}.items(),
        condition=UnlessCondition(use_gui),
    )

    # Spawn
    
    stretch_sdf = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('stretch_gz_sim'),
            'urdf',
            'stretch_re1',
            'stretch_sim_re1.sdf'
        ])
    ])

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'stretch',
            '-string', stretch_sdf,
            '-z', '0.1',
        ],
        output='screen',
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Lidar (Gazebo -> ROS2)
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Base IMU (Gazebo -> ROS2)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Wrist Accelerometer (Gazebo -> ROS2)
            '/wrist_imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # RGBD Camera (Gazebo -> ROS2)
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ("/imu", "imu/data"),
            ("/wrist_imu", "wrist_imu/data"),
        ],
        output='screen'
    )

    # Controllers
    joints_config_ = os.path.join(pkg_stretch_gz_sim, 'config', 'joints.yaml')
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['stretch_joint_state_controller',
            '--param-file',
            joints_config_,
        ],
    )
    drive_config_ = os.path.join(pkg_stretch_gz_sim, 'config', 'drive_config.yaml')
    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'stretch_diff_drive_controller',
            '--param-file',
            drive_config_,
            '--controller-ros-args',
            '-r /diff_drive_controller/cmd_vel:=/cmd_vel',
        ],
    )
    stretch_arm_controller_config_ = os.path.join(pkg_stretch_gz_sim, 'config', 'arm.yaml')
    stretch_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'stretch_arm_controller',
            '--param-file',
            stretch_arm_controller_config_,
        ],
    )
    stretch_head_controller_config_ = os.path.join(pkg_stretch_gz_sim, 'config', 'head.yaml')
    stretch_head_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'stretch_head_controller',
            '--param-file',
            stretch_head_controller_config_,
        ],
    )
    stretch_gripper_controller_config_ = os.path.join(pkg_stretch_gz_sim, 'config', 'gripper.yaml')
    stretch_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'stretch_gripper_controller',
            '--param-file',
            stretch_gripper_controller_config_,
        ],
        )

    # Sensor Static TFs
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_transform_publisher',
        output='log',
        arguments=['0', '0.0', '0.1664', '0.0', '0.0', '0.0', 'base_link', 'stretch/link_laser/lidar']
    )
    rgbd_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rgbd_static_transform_publisher',
        output='log',
        arguments=[
            '0.0', '0.0', '0.0', '1.5708', '-1.5708', '0',
            'camera_depth_optical_frame', 'camera_link_optical'
        ]
    )

    # Node to bridge camera image with image_transport and compressed_image_transport
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'camera.image.compressed.jpeg_quality': 75},
        ],
    )
    # # Relay node to republish camera_info to /camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Use base footprint
    base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rgbd_static_transform_publisher',
        output='log',
        arguments=[
            '0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
            'base_footprint', 'base_link'
        ]
    )

    return LaunchDescription(
        [
            # Launch Arguments
            DeclareLaunchArgument(
                'use_sim_time',
                default_value=use_sim_time,
                description="If true, use simulated clock"),

            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[diff_drive_base_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=diff_drive_base_controller_spawner,
                    on_exit=[stretch_arm_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=stretch_arm_controller_spawner,
                    on_exit=[stretch_head_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=stretch_head_controller_spawner,
                    on_exit=[stretch_gripper_controller_spawner],
                )
            ),
            declare_world_cmd,
            declare_use_gui_cmd,
            declare_use_base_footprint_cmd,
            # Nodes and Launches
            gazebo,
            gazebo_headless,
            spawn,
            bridge,
            robot_state_publisher,
            lidar_static_tf,
            rgbd_static_tf,
            gz_image_bridge_node,
            relay_camera_info_node,
            base_footprint,
        ]
    )

