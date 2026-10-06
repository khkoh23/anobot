from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    executor = Node(
        package="anobot_bt",
        executable="anobot_task_executor",
        name="anobot_task_executor",
        output="screen",
    )

    return LaunchDescription([
        executor,
    ])