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
        arguments=['/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )
    
    thruster_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='thruster_bridge',
        arguments=['/model/tethys/joint/propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    
    vertical_fin_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vertical_fin_bridge',
        arguments=['/model/tethys/joint/vertical_fins_joint/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    
    horizontal_fin_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='horizontal_fin_bridge',
        arguments=['/model/tethys/joint/horizontal_fins_joint/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
        
    teleop_node = Node(
        package='submarine_control',
        executable='submarine_teleop',
        name='submarine_teleop',
        output='screen',
        prefix='xterm -e'
    )

    ld = LaunchDescription()
    ld.add_action(set_gazebo_resource_path_action)
    ld.add_action(gazebo_launch)
    ld.add_action(camera_bridge)
    ld.add_action(imu_bridge)
    ld.add_action(thruster_bridge)
    ld.add_action(vertical_fin_bridge)
    ld.add_action(horizontal_fin_bridge)
    ld.add_action(teleop_node)
    
    return ld