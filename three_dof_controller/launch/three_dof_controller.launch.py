import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('three_dof_controller'),
        'urdf',
        'three_dof.urdf')

    rviz_config_file = os.path.join(
        get_package_share_directory('three_dof_controller'),
        'rviz',
        'urdf.rviz')

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen', parameters=[{'robot_description': open(urdf_file).read()}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher', name='joint_state_publisher', output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', rviz_config_file]),
        Node(package='three_dof_controller', executable='controller', name='three_dof_controller', output='screen')
    ])