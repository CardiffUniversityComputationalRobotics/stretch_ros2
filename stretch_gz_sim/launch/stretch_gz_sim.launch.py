import os
import xml.etree.ElementTree as ET
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
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


def _as_bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


DIFF_DRIVE_PLUGIN = 'gz::sim::systems::DiffDrive'
LINK_VELOCITY_PLUGIN = 'link_velocity_plugin::LinkVelocityPlugin'


def _normalize_drive_system(value):
    drive_system = value.strip().lower().replace('-', '_')

    if drive_system in ('diff', 'diff_drive', 'diffdrive'):
        return 'diff_drive'

    if drive_system in (
        'link_velocity',
        'link_velocity_plugin',
        'velocity',
        'velocity_control',
        'gz_sim_velocity_control_system',
    ):
        return 'link_velocity'

    raise RuntimeError(
        "Unsupported drive_system "
        f"'{value}'. Use 'diff_drive' or 'link_velocity'."
    )


def _set_diff_drive_tf_topic(plugin, tf_topic):
    tf_topic_elem = plugin.find('tf_topic')
    if tf_topic_elem is None:
        tf_topic_elem = ET.SubElement(plugin, 'tf_topic')
    tf_topic_elem.text = tf_topic


def _render_stretch_sdf(sdf_path, tf_topic, drive_system):
    root = ET.parse(sdf_path).getroot()
    drive_system = _normalize_drive_system(drive_system)
    selected_plugin = (
        DIFF_DRIVE_PLUGIN
        if drive_system == 'diff_drive'
        else LINK_VELOCITY_PLUGIN
    )
    drive_plugins_found = {
        DIFF_DRIVE_PLUGIN: False,
        LINK_VELOCITY_PLUGIN: False,
    }

    for parent in root.iter():
        for plugin in list(parent.findall('plugin')):
            plugin_name = plugin.get('name')
            if plugin_name not in drive_plugins_found:
                continue

            drive_plugins_found[plugin_name] = True

            if plugin_name != selected_plugin:
                parent.remove(plugin)
                continue

            if plugin_name == DIFF_DRIVE_PLUGIN:
                _set_diff_drive_tf_topic(plugin, tf_topic)

    missing_plugins = [
        plugin_name
        for plugin_name, was_found in drive_plugins_found.items()
        if not was_found
    ]
    if missing_plugins:
        raise RuntimeError(
            'Failed to find drive plugin(s) in stretch SDF: '
            + ', '.join(missing_plugins)
        )

    return ET.tostring(root, encoding='unicode')


def _create_spawn_node(context, stretch_sdf_path, model_name):
    enable_odom_tf = _as_bool(
        LaunchConfiguration('enable_odom_tf').perform(context)
    )
    tf_topic = f'/model/{model_name}/tf' if enable_odom_tf else '/disabled_odom_tf'
    drive_system = LaunchConfiguration('drive_system').perform(context)
    stretch_sdf = _render_stretch_sdf(stretch_sdf_path, tf_topic, drive_system)

    return [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', model_name,
                '-string', stretch_sdf,
                '-z', '0.1',
            ],
            output='screen',
        )
    ]

def generate_launch_description():
    # Main package
    pkg_stretch_gz_sim = get_package_share_directory('stretch_gz_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    model_name = 'stretch'

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


    spawn = OpaqueFunction(
        function=lambda context: _create_spawn_node(
            context,
            stretch_sdf_path,
            model_name,
        )
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Velocity commands (ROS2 -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
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

    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(pkg_stretch_gz_sim, 'rviz', 'default.rviz')]]
    )

    return LaunchDescription(
        [
            # Launch Arguments
            DeclareLaunchArgument(
                'use_sim_time',
                default_value=use_sim_time,
                description="If true, use simulated clock"),
            DeclareLaunchArgument(
                'enable_odom_tf',
                default_value='true',
                description='If true, publish DiffDrive odom TF on /model/stretch/tf.'),
            DeclareLaunchArgument(
                'drive_system',
                default_value='diff_drive',
                description=(
                    "Drive system to use: 'diff_drive' or "
                    "'link_velocity'."
                )),
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
            rviz_node,
        ]
    )
