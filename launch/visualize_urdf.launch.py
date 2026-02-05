import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    #######################
    # Package Directories #
    #######################

    anymal_description_path = get_package_share_directory("anymal_description")
    # go2_description_path = get_package_share_directory("go2_description")
    
    # mmp_quadruped_path = get_package_share_directory("mmp_quadruped")
    reachability_analysis_path = get_package_share_directory("reachability_analysis")

    ####################
    # Launch Arguments #
    ####################
    urdfFile = os.path.join(anymal_description_path, "urdf/anymal.urdf")

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=urdfFile
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'publish_frequency': 100.0
                },
                {
                    'use_tf_static': True
                }
            ],
            arguments=[launch.substitutions.LaunchConfiguration("urdfFile")],
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_gui': True
                },
                {
                    'rate': 100.0
                }
            ]
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                os.path.join(
                    reachability_analysis_path, "rviz", "reachability_gui.rviz",
                )
            ],
            parameters=[
                {
                    "use_sim_time": True,
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
