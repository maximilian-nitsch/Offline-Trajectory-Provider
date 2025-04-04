"""@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
"""

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = "offline_trajectory_provider_package"

    # Get the path to the nanoAUV URDF file
    urdf_file_nanoauv = get_package_share_directory(package_name) + '/urdf/' \
        'nanoauv_green.urdf'

    # # Get the path to the LRS URDF file
    # urdf_file_lrs = get_package_share_directory(package_name) + '/urdf/' \
    #     'lrs.urdf'

    # # RViz2 configuration file
    # rviz_config_file = get_package_share_directory(package_name) + '/rviz/' \
    #     'nanoauv.rviz'

    # Create the launch description
    ld = LaunchDescription()
    
    # Create the Robot State Publisher node for nanoAUV
    nanoauv_description_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="navigation",
        name="nanoauv_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': False}],
        arguments=[urdf_file_nanoauv]
    )

    ld.add_action(nanoauv_description_node)

    # # Create the RViz2 node
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=['-d', rviz_config_file]  # Load the RViz2 configuration file
    # )

    # ld.add_action(rviz_node)

    return ld
