"""
Launch file for Flexiv Rizon robot with Cartesian motion-force control.
This launch file sets up the robot in RT_CARTESIAN_MOTION_FORCE mode for
real-time Cartesian pure motion control.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    rizon_type_param_name = "rizon_type"
    robot_sn_param_name = "robot_sn"
    start_rviz_param_name = "start_rviz"
    load_mounted_ft_sensor_param_name = "load_mounted_ft_sensor"
    use_fake_hardware_param_name = "use_fake_hardware"
    fake_sensor_commands_param_name = "fake_sensor_commands"

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            rizon_type_param_name,
            description="Type of the Flexiv Rizon robot.",
            default_value="Rizon4",
            choices=["Rizon4", "Rizon4M", "Rizon4R", "Rizon4s", "Rizon10", "Rizon10s"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_sn_param_name,
            description="Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            start_rviz_param_name,
            default_value="true",
            description="Start RViz automatically with the launch file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            load_mounted_ft_sensor_param_name,
            default_value="false",
            description="Flag to load the mounted force torque sensor.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            use_fake_hardware_param_name,
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            fake_sensor_commands_param_name,
            default_value="false",
            description="Enable fake command interfaces for sensors.",
        )
    )

    # Initialize Arguments
    rizon_type = LaunchConfiguration(rizon_type_param_name)
    robot_sn = LaunchConfiguration(robot_sn_param_name)
    start_rviz = LaunchConfiguration(start_rviz_param_name)
    load_mounted_ft_sensor = LaunchConfiguration(load_mounted_ft_sensor_param_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_param_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_param_name)

    # Fixed to cartesian_motion_force mode
    rdk_control_mode = "cartesian_motion_force"

    # Get URDF via xacro
    flexiv_urdf_xacro = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "urdf", "rizon.urdf.xacro"]
    )

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                flexiv_urdf_xacro,
                " ",
                "robot_sn:=",
                robot_sn,
                " ",
                "rizon_type:=",
                rizon_type,
                " ",
                "rdk_control_mode:=",
                rdk_control_mode,
                " ",
                "load_gripper:=false",
                " ",
                "load_mounted_ft_sensor:=",
                load_mounted_ft_sensor,
                " ",
                "use_fake_hardware:=",
                use_fake_hardware,
                " ",
                "fake_sensor_commands:=",
                fake_sensor_commands,
            ]
        ),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    # RViZ
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "rviz", "view_rizon.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    # Robot controllers for Cartesian mode
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("flexiv_bringup"), "config", "rizon_cartesian_controllers.yaml"]
    )

    # Controller Manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(robot_controllers, allow_substs=True),
            {"robot_sn": robot_sn},
            {"rdk_control_mode": rdk_control_mode},
        ],
        remappings=[("joint_states", "flexiv_arm/joint_states")],
        output="both",
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": ["flexiv_arm/joint_states"],
                "rate": 30,
            }
        ],
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Run joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Run Flexiv robot states broadcaster
    flexiv_robot_states_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flexiv_robot_states_broadcaster"],
        parameters=[{"robot_sn": robot_sn}],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Run Cartesian motion controller
    cartesian_motion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_motion_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of cartesian_motion_controller after joint_state_broadcaster
    delay_cartesian_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[cartesian_motion_controller_spawner],
        )
    )

    # Delay rviz start after cartesian_motion_controller_spawner
    delay_rviz_after_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cartesian_motion_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        ros2_control_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        flexiv_robot_states_broadcaster_spawner,
        delay_cartesian_controller_spawner,
        delay_rviz_after_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
