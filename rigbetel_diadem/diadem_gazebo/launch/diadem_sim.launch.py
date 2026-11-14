import os
import shutil
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_gazebo_share = get_package_share_directory('diadem_gazebo')
    pkg_desc_share = get_package_share_directory('diadem_description')

    worlds_dir = os.path.join(pkg_gazebo_share, 'worlds')
    xacro_file = os.path.join(pkg_desc_share, 'urdf', 'diadem_sim.xacro')

    gz_bridge_core_config = os.path.join(pkg_gazebo_share, 'config', 'gz_bridge_core.yaml')
    gz_bridge_scan_config = os.path.join(pkg_gazebo_share, 'config', 'gz_bridge_scan.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    # Convert xacro to robot_description parameter
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # Gazebo Sim runner
    def launch_gazebo(context, *args, **kwargs):
        world_val = world_name.perform(context)
        if os.path.isabs(world_val):
            world_path = world_val
        else:
            world_path = os.path.join(worlds_dir, world_val)

        gz_bin = 'gz' if shutil.which('gz') else 'ign'
        sub_cmd = 'sim' if gz_bin == 'gz' else 'gazebo'

        cmd = [gz_bin, sub_cmd, '-r', world_path, '--verbose', '1']
        return [ExecuteProcess(cmd=cmd, output='screen')]

    gazebo_process = OpaqueFunction(function=launch_gazebo)

    # Spawn Diadem Entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_diadem',
        arguments=[
            '-name', 'diadem',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Core Parameter Bridge (cmd_vel, odom, imu, clock)
    bridge_core = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_core',
        arguments=['--ros-args', '-p', ['config_file:=', gz_bridge_core_config]],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Scan Parameter Bridge (laser scan)
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_scan',
        arguments=['--ros-args', '-p', ['config_file:=', gz_bridge_scan_config]],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='simple_warehouse.sdf',
            description='World file name (e.g. simple_warehouse.sdf, nav2_test_world.sdf, room2.sdf)'
        ),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='Initial X position'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial Y position'),
        DeclareLaunchArgument('z_pose', default_value='0.2', description='Initial Z position'),
        DeclareLaunchArgument('yaw_pose', default_value='0.0', description='Initial Yaw orientation'),

        # Expose resources (worlds & meshes) to Gazebo
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', f"{worlds_dir}:{pkg_desc_share}/.."),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', f"{worlds_dir}:{pkg_desc_share}/.."),

        gazebo_process,
        robot_state_publisher,
        spawn_entity,
        bridge_core,
        bridge_scan,
    ])
