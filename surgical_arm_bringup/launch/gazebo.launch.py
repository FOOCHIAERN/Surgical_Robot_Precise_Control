import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'surgical_arm_bringup'
    urdf_file = 'FYP_URDF.urdf' # <-- Ensure this matches your actual URDF name

    urdf_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', urdf_file)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 1. Set Gazebo Resource Path (CRUCIAL FOR VISIBLE MESHES)
    # This tells Gazebo exactly where your install folder is so it can resolve "package://" URIs
    install_dir = get_package_prefix(pkg_name)
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(install_dir, 'share')
    )

    # 2. Start robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. Include Gazebo Harmonic (Added '-r' to auto-unpause the simulation)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    # 4. Spawn the robot using the topic (More reliable than passing the raw string)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'surgical_arm',
                   '-allow_renaming', 'true'],
        output='screen'
    )

    return LaunchDescription([
        set_resource_path,
        rsp_node,
        gazebo,
        spawn_entity
    ])