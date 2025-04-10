# IRT C++/ROS 2 nanoAUV Offline Trajectory Provider

[![License](https://img.shields.io/badge/license-BSD--3-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

<!--- protected region package header begins -->
**Author:**
- Maximilian Nitsch <maximilian.nitsch@rwth-aachen.de> (Institute of Automatic Control - RWTH Aachen University)

**Maintainer:**
  - Maximilian Nitsch <maximilian.nitsch@rwth-aachen.de> (Institute of Automatic Control - RWTH Aachen University)
<!--- protected region package header ends -->

## Description
This project implements a provider for three different trajectories of the autonomous underwater vehicle "nanoAUV".

The simulator implements the following features:
- "Lawnmower" trajectory
- "YoYo" trajectory
- "Validation" trajectory (mix of different trajectories)

Our in-house hydrodynamics and control simulator have generated all trajectories. The trajectories are provided as CSV files.
The Offline Trajectory Provides loads a CSV file and publishes the data as ROS 2 odometry and AccelStamped message.

## Table of Contents

- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [ROS 2 Nodes](#ros-2-nodes)
  - [Publishers](#publisher-node)
- [Coding Guidelines](#coding-guidelines)
- [References](#references)
- [Contributing](#contributing)
- [License](#license)

# Dependencies

This project depends on the following literature and libraries:

- **ROS 2 Humble**: ROS 2 is a set of software libraries and tools for building robot applications: [ROS 2 Installation page](https://docs.ros.org/en/humble/Installation.html)).


# Installation

To install the `offline_trajectory_provider_package`, you need to follow these steps:

1. **Install ROS 2 Humble**: Make sure you have ROS 2 (Humble) installed. You can follow the official installation instructions provided by ROS 2. Visit [ROS 2 Humble Installation page](https://docs.ros.org/en/humble/Installation.html) for detailed installation instructions tailored to your platform.

3. **Clone the Package**: Clone the package repository to your ROS 2 workspace. If you don't have a ROS 2 workspace yet, you can create one using the following commands:

    ```bash
    mkdir -p /path/to/ros2_workspace/src
    cd /path/to/ros2_workspace/src
    ```

    Now, clone the package repository:

    ```bash
    git clone <repository_url>
    ```

    Replace `<repository_url>` with the URL of your package repository.

4. **Build the Package**: Once the package is cloned, you must build it using colcon, the default build system for ROS 2. Navigate to your ROS 2 workspace and run the following command:

    ```bash
    cd /path/to/ros2_workspace
    colcon build
    ```

    This command will build all the packages in your workspace, including the newly added package.

5. **Source the Workspace**: After building the package, you need to source your ROS 2 workspace to make the package available in your ROS 2 environment. Run the following command:

    ```bash
    source /path/to/ros2_workspace/install/setup.bash
    ```

    Replace `/path/to/ros2_workspace` with the actual path to your ROS 2 workspace.

That's it! Your `offline_trajectory_provider_package` should now be installed along with its dependencies and ready to use in your ROS 2 environment.

## Usage

1. **Configure your YAML file** for your IMU or use the default file.

2. **Start the Offline Trajectory Provider** with the launch file:
    ```bash
    ros2 launch offline_trajectory_provider_package offline_trajectory_provider_launch.launch.py
    ```
  The Offline Trajectory Provider now starts reading in the selected CSV file and publishes ground truth odometry and acceleration messages.

## ROS 2 Nodes

The IMU simulator node implements five publishers and subscribes to one topic.
ROS 2 services or actions are not provided.

### Publishers

This node publishes the following topics:

| Topic Name       | Message Type        | Description                        |
|------------------|---------------------|------------------------------------|
| `/auv /odometry`   | `nav_msgs/Odometry.msg`   | Publishes odometry data.|
| `auv/accel`   | `geometry_msgs/AccelStamped.msg`   | Publishes acceleration data.|

## Coding Guidelines

This project follows these coding guidelines:
- https://google.github.io/styleguide/cppguide.html
- http://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html 

## References

The offline trajectory provider simulator implementation closely follows the work:
- M. Nitsch, "Navigation of a miniaturized autonomous underwater vehicle exploring waters under ice," Dissertation, Rheinisch-Westfälische Technische Hochschule Aachen, Aachen, RWTH Aachen University, 2024. [DOI: 10.18154/RWTH-2024-05964](https://www.researchgate.net/publication/382562855_Navigation_of_a_Miniaturized_Autonomous_Underwater_Vehicle_Exploring_Waters_Under_Ice?_sg%5B0%5D=xNyP6RXVcEfazembhPB6cRxGQTBAvWqw6qMza26FExHUVzWcV9VUd35T4l6KjUqbo1a7W6okgPi3zqqUQYww5dmfZgsQcoJlvBE3ss1T.JLcM4K_iQyfJO7N73P9ebOmEd0xchppKYQemo5hh6ecobLxw5ZSaPgwEvlqYcQtr25iVtPvdMorpxfHK_Oldag&_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6ImhvbWUiLCJwYWdlIjoicHJvZmlsZSIsInBvc2l0aW9uIjoicGFnZUNvbnRlbnQifX0).

## Contributing

If you want to contribute to the project, see the [CONTRIBUTING](CONTRIBUTING) file for details.

## License

This project is licensed under the BSD-3-Clause License. See the [LICENSE](LICENSE) file for details.
