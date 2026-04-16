import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define the package name and the URDF file name
    pkg_name = 'surgical_arm_bringup'
    urdf_file = 'FYP_URDF.urdf' # <-- UPDATE THIS TO YOUR ACTUAL URDF FILE NAME

    # 2. Get the exact path to the URDF file
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        urdf_file
    )

    # 3. Read the URDF file contents
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 4. Node: robot_state_publisher 
    # Broadcasts the physical properties and transforms of your arm to the ROS network
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 5. Node: joint_state_publisher_gui 
    # Generates a separate UI window with sliders to manually actuate your 3 translational joints
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 6. Node: rviz2 
    # Launches the 3D visualization environment
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 7. Execute all nodes
    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])