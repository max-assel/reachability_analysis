import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration



def generate_launch_description():

    #######################
    # Package Directories #
    #######################

    anymal_description_path = get_package_share_directory("anymal_description")
    
    # mmp_quadruped_path = get_package_share_directory("mmp_quadruped")
    reachability_analysis_path = get_package_share_directory("reachability_analysis")

    ####################
    # Launch Arguments #
    ####################
    urdfFile = os.path.join(anymal_description_path, "urdf/anymal.urdf")

    taskFile = os.path.join(reachability_analysis_path, "config/task.info")
    frameFile = os.path.join(reachability_analysis_path, "config/frame_declaration.info")

    declare_description_name = DeclareLaunchArgument(
            "description_name",
            default_value=urdfFile
    )

    ##########
    ## Node ##
    ##########
    world_to_odom_tf2_node = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_tf2",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "world",
            "odom"
        ],
        parameters=[
            {
                "use_sim_time": True,
            }
        ]
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                reachability_analysis_path, "rviz", "reachability.rviz",
            )
        ],
        parameters=[
            {
                "use_sim_time": True,
            }
        ]
    )

    # Subscribe to joint states and publish TFs
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output='screen',
        arguments=[LaunchConfiguration("description_name")],
        parameters=[
            {
                "use_sim_time": True,
                "publish_frequency": 200.0,
                "ignore_timestamp": True
            }
        ]
    )

    rqt_node = Node(
        package="rqt_reconfigure",
        executable="rqt_reconfigure",
        name="rqt_reconfigure",
        output="screen"
    )

    reachability_analysis_node = launch_ros.actions.Node(
        package="reachability_analysis",
        executable="legged_robot_reachability_analysis",
        name="legged_robot_reachability_analysis",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                'taskFile': taskFile,
                'frameFile': frameFile,
                'urdfFile': urdfFile,
            }
        ]
    )

    decl_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=True)

    ###########################
    # Full Launch Description #
    ###########################
    return LaunchDescription(
            [
                decl_sim_time,
                declare_description_name,
                world_to_odom_tf2_node,
                robot_state_publisher_node, 
                rqt_node,
                rviz_node,
                reachability_analysis_node
            ]
    )