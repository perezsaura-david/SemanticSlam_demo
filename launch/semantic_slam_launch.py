""" Launch file for semantic SLAM. """

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """ Launch semantic SLAM node. """

    return LaunchDescription([
        Node(
            package='as2_slam',
            executable='as2_slam_node',
            name='semantic_slam_node',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'odometry_topic': '/drone0/noisy_odometry'},
                        # {'aruco_pose_topic': '/drone0/detect_aruco_markers_behavior/aruco_pose'},
                        {'aruco_pose_topic': '/drone0/processed_gate_poses'},
                        {'map_frame': 'drone0/map'},
                        {'odom_frame': 'drone0/odom'},
                        {'robot_frame': 'drone0/base_link'},
                        {'viz_main_markers_topic': 'slam_viz/markers/main'},
                        {'viz_temp_markers_topic': 'slam_viz/markers/temp'},
            ],
            emulate_tty=True,
        ),
    ])
