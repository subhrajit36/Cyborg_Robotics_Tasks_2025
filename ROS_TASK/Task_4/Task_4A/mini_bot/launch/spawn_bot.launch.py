''' 
*****************************************************************************************
*
*        =============================================
*                  Cyborg ROS Task-2
*        =============================================
*
*
*  Filename:		spawn_bot.launch.py
*  Description:         Use this file to spawn bot.
*  Created:		16/07/2023
*  Last Modified:	04/07/2024
*  Modified by:         Soumitra Naik
*  Author:		e-Yantra Team (Srivenkateshwar)
*  
*****************************************************************************************
'''

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription ,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration, PythonExpression
import os
import xacro
from ament_index_python.packages import get_package_share_directory,get_package_prefix


def generate_launch_description():
    share_dir = get_package_share_directory('mini_bot')
    xacro_file = os.path.join(share_dir, 'urdf','mini_bot_1.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
  
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mini_bot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        urdf_spawn_node
    ])
