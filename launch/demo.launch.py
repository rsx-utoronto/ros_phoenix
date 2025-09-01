import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name="PhoenixContainer",
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        parameters=[{"interface": "can0"}],
        composable_node_descriptions=[
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="front_left",
                parameters=[{"id": 4}],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="front_right",
                parameters=[{"id": 1}],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="back_left",
                parameters=[{"id": 6}],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="back_right",
                parameters=[{"id": 3}],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription([container,
                                     Node(
                                         package="ros_phoenix",
                                         executable="falcon_motor_control_node",
                                         name="falcon_motor_control_node",
                                         output="screen",
                                         respawn=False
                                     )])
