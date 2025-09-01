import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


def generate_launch_description():
    # Inline config previously in goose.yaml
    talons_cfg = {
        "back_left":  {"id": 6, "P": 3.0, "I": 0.015, "D": 175.0, "invert": False},
        "back_right": {"id": 3, "P": 3.0, "I": 0.015, "D": 175.0, "invert": True},
        # "mid_left":   {"id": 5, "P": 3.0, "I": 0.015, "D": 175.0, "invert": False},
        # "mid_right":  {"id": 2, "P": 3.0, "I": 0.015, "D": 175.0, "invert": True},
        "front_left": {"id": 4, "P": 3.0, "I": 0.015, "D": 175.0, "invert": False},
        "front_right":{"id": 1, "P": 3.0, "I": 0.015, "D": 175.0, "invert": True},
    }

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
                parameters=[talons_cfg["front_left"]],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="front_right",
                parameters=[talons_cfg["front_right"]],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="back_left",
                parameters=[talons_cfg["back_left"]],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="back_right",
                parameters=[talons_cfg["back_right"]],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription([
        container,
        Node(
            package="ros_phoenix",
            executable="falcon_motor_control_node",
            name="falcon_motor_control_node",
            output="screen",
            respawn=False
        ),
    ])
