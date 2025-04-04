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

    # Declare the launch arguments
    topic_name_odom_arg = DeclareLaunchArgument(
        "topic_name_odom",
        default_value="/navigation/odometry",
        description="Topic name of the ground truth odometry from vehicle",
    )

    topic_name_accel_arg = DeclareLaunchArgument(
        "topic_name_accel",
        default_value="/navigation/accel",
        description="Topic name of the ground truth acceleration from vehicle",
    )

    base_link_frame_id_arg = DeclareLaunchArgument(
        "base_link_frame_id",
        default_value="base_link_sname_estimated",
        description="Frame ID of the estimated base link of the vehicle",
    )

    csv_file_path_arg = DeclareLaunchArgument(
        "csv_file_path",
        default_value=get_package_share_directory(package_name),
        description="Path to the directory containing the CSV file "
                    "with the ground truth data",
    )

    csv_file_name_arg = DeclareLaunchArgument(
        "csv_file_name",
        default_value="trajectory_validation_esekf.csv",
        description="Name of the CSV file containing the ground truth data",
    )

    verbose_level_arg = DeclareLaunchArgument(
        "verbose_level",
        default_value="1",
        description="Level of verbosity of the node",
    )

    # Add the launch argument to the launch description
    ld.add_action(topic_name_odom_arg)
    ld.add_action(topic_name_accel_arg)
    ld.add_action(base_link_frame_id_arg)
    ld.add_action(csv_file_path_arg)
    ld.add_action(csv_file_name_arg)
    ld.add_action(verbose_level_arg)

    # Create the node
    offline_trajectory_provider_node = Node(
        package="offline_trajectory_provider_package",
        executable="offline_trajectory_provider_package_node",
        name="nanoauv_trajectory_provider_node",
        output="screen",
        parameters=[
            {"topic_name_odom": LaunchConfiguration('topic_name_odom')},
            {"topic_name_accel": LaunchConfiguration("topic_name_accel")},
            {"base_link_frame_id": LaunchConfiguration("base_link_frame_id")},
            {"csv_file_path": LaunchConfiguration("csv_file_path")},
            {"csv_file_name": LaunchConfiguration("csv_file_name")},
            {"verbose_level": LaunchConfiguration("verbose_level")}
        ]
    )

    ld.add_action(offline_trajectory_provider_node)

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
