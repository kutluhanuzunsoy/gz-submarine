from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_submarine_gazebo_share = get_package_share_directory('submarine_gazebo')
    world_path = os.path.join(pkg_submarine_gazebo_share, 'worlds', 'submarine_world.sdf')

    submarine_models_path = os.path.join(pkg_submarine_gazebo_share, 'models')
    current_gz_sim_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    if current_gz_sim_resource_path:
        new_gz_sim_resource_path = submarine_models_path + os.pathsep + current_gz_sim_resource_path
    else:
        new_gz_sim_resource_path = submarine_models_path
    
    set_gazebo_resource_path_action = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_sim_resource_path
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )
    imu_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='imu_bridge',
    arguments=['/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU'], # Bridge IMU data
    output='screen'
    )
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    ld = LaunchDescription()
    ld.add_action(set_gazebo_resource_path_action)
    ld.add_action(gazebo_launch)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(camera_bridge)
    ld.add_action(teleop_node)
    ld.add_action(imu_bridge)
    return ld