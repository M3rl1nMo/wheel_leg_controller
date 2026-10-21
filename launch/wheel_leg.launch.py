"""一键启动：仿真 + 控制器 + 手柄驱动

用法：
    ros2 launch wheel_leg_controller wheel_leg.launch.py
不用手柄时：
    ros2 launch wheel_leg_controller wheel_leg.launch.py use_joy:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_joy = LaunchConfiguration('use_joy')

    return LaunchDescription([
        # 启动参数：是否启动手柄驱动，默认启动
        DeclareLaunchArgument('use_joy', default_value='true'),

        # 1. 仿真节点
        Node(
            package='wheel_leg_sim',
            executable='wheel_leg_sim',
            output='screen',
        ),

        # 2. 控制器节点
        Node(
            package='wheel_leg_controller',
            executable='wheel_leg_controller',
            output='screen',
        ),

        # 3. 手柄驱动，只有 use_joy 为 true 时才启动
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            condition=IfCondition(use_joy),
        ),
    ])
