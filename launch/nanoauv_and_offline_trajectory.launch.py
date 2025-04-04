"""@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "offline_trajectory_provider_package"
    # Get the directory of the package containing your launch files
    my_package_dir = get_package_share_directory(package_name)

    # Define the launch arguments
    topic_name_odom_arg = ('topic_name_odom', '/nanoauv/odometry')
    topic_name_accel_arg = ('topic_name_accel', '/nanoauv/accel')
    base_link_frame_id_arg = ('base_link_frame_id', 'base_link_sname')
    csv_file_path_arg = ('csv_file_path', my_package_dir)
    csv_file_name_arg = ('csv_file_name', 'trajectory_validation.csv')
    trajectory_rate_arg = ('trajectory_rate', '250.0')
    verbose_level_arg = ('verbose_level', '0')

    # Launch the first launch file
    offline_trajectory_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([my_package_dir, '/launch/offline_trajectory_provider_launch.launch.py']),
        launch_arguments=[topic_name_odom_arg, topic_name_accel_arg, base_link_frame_id_arg, csv_file_path_arg, csv_file_name_arg, trajectory_rate_arg, verbose_level_arg]
    )

    topic_name_odom_nav_arg = ('topic_name_odom', '/navigation/odometry')
    topic_name_accel_nav_arg = ('topic_name_accel', '/navigation/accel')
    base_link_frame_id_nav_arg = ('base_link_frame_id', 'base_link_sname_estimated')
    csv_file_name_nav_arg = ('csv_file_name', 'trajectory_validation_esekf.csv')
    trajectory_rate_arg = ('trajectory_rate', '100.0')

    # Launch the second launch file
    navigation_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([my_package_dir, '/launch/navigation_filter_trajectory_provider.launch.py']),
        launch_arguments=[topic_name_odom_nav_arg, topic_name_accel_nav_arg, base_link_frame_id_nav_arg, csv_file_path_arg, csv_file_name_nav_arg, trajectory_rate_arg, verbose_level_arg]
    )

    # Add both launch actions to the launch description
    return LaunchDescription([
        offline_trajectory_launch,
        navigation_filter_launch
    ])
