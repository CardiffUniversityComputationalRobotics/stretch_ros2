import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Main package
    pkg_stretch_gz_sim = get_package_share_directory('stretch_gz_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

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

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Gazebo Sim
    world = LaunchConfiguration("world")

    world_path = os.path.join(pkg_stretch_gz_sim, 'worlds', 'empty_world.sdf')
    
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value= world_path,
        description="Path of the world to show.",
    )

    world_str_path = [TextSubstitution(text='-r '), world]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_str_path}.items(),
    )

    # Spawn
    stretch_sdf_path = os.path.join(pkg_stretch_gz_sim, "urdf", "stretch_re1" ,"stretch_re1.sdf")

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'stretch',
            '-file', stretch_sdf_path,
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
            # Velocity commands (ROS2 -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # JointTrajectory bridge (ROS2 -> Gazebo)
            '/joint_trajectory@trajectory_msgs/msg/JointTrajectory@gz.msgs.JointTrajectory',
            # Odometry (Gazebo -> ROS2)
            '/model/stretch/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # odom->base_link tf (Gazebo -> ROS2)
            '/model/stretch/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            '/world/default/model/stretch/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # JointTrajectoryProgress bridge (Gazebo -> ROS2)
            '/joint_trajectory_progress@std_msgs/msg/Float32[gz.msgs.Float',
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
            ("/model/stretch/tf", "tf"),
            ("/world/default/model/stretch/joint_state", "joint_states"),
            ("/model/stretch/odometry", "odom"),
            ("/imu", "imu/data"),
            ("/wrist_imu", "wrist_imu/data"),
        ],
        output='screen'
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

    return LaunchDescription(
        [
            # Launch Arguments
            DeclareLaunchArgument(
                'use_sim_time',
                default_value=use_sim_time,
                description="If true, use simulated clock"),
            declare_world_cmd,
            # Nodes and Launches
            gazebo,
            spawn,
            bridge,
            robot_state_publisher,
            lidar_static_tf,
            rgbd_static_tf,
            gz_image_bridge_node,
            relay_camera_info_node,
        ]
    )

