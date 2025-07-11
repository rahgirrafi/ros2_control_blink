from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    robot_description_content = '/run/media/rafi/Technical/Github/ws_ros2_control_blink/install/arduino_hardware/share/arduino_hardware/resource/arduino_led.urdf'

    with open(robot_description_content, 'r') as infp:
        robot_description = infp.read()

    robot_description = {"robot_description": robot_description}
    robot_controllers = PathJoinSubstitution(
            [
                FindPackageShare("arduino_hardware"),
                "config",
                "controllers.yaml",
            ]
        )

    control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_controllers],
            output="both",
            remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )
    

    joint_state_broadcaster_spawn = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )





    ld = LaunchDescription()
    ld.add_action(control_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster_spawn)
    ld.add_action(robot_controller_spawner)
    return ld

