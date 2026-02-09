from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="fleetops_heartbeat",
            executable="heartbeat_publisher",
            name="heartbeat_publisher",
            output="screen",
            parameters=[
                {"robot_id": "robot_1"},
                {"publish_hz": 5.0},
            ],
        ),
        Node(
            package="fleetops_health",
            executable="health_supervisor",
            name="health_supervisor",
            output="screen",
            parameters=[
                {"timeout_sec": 2.0},
            ],
        ),
    ])