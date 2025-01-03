# SPDX-FileCopyrightText: 2025 Chuma Naoki
# SPDX-License-Identifier: BSD-3-Clause 
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():

    monte_carlo_publisher = launch_ros.actions.Node(
            package='mypkg',
            executable='monte_carlo_publisher',
            )
    result = launch_ros.actions.Node(
            package='mypkg',
            executable='result',
            output='screen'
            )

    return launch.LaunchDescription([monte_carlo_publisher, result])

