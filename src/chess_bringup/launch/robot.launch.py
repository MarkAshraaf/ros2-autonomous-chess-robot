from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # Launch arguments
    # ============================================================

    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur5e",
        description="UR robot type"
    )

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.102",
        description="UR robot IP address"
    )

    gripper_ip_arg = DeclareLaunchArgument(
        "gripper_ip",
        default_value="192.168.1.102",
        description="Robot IP address used by the Robotiq gripper adapter"
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="true for fake/simulation hardware, false for real robot"
    )

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch RViz with MoveIt"
    )

    velocity_scale_arg = DeclareLaunchArgument(
        "velocity_scale",
        default_value="0.55",
        description="MoveIt velocity scaling factor"
    )

    acceleration_scale_arg = DeclareLaunchArgument(
        "acceleration_scale",
        default_value="0.50",
        description="MoveIt acceleration scaling factor"
    )

    gripper_wait_sec_arg = DeclareLaunchArgument(
        "gripper_wait_sec",
        default_value="0.5",
        description="Wait time after each gripper command"
    )

    startup_calibration_enabled_arg = DeclareLaunchArgument(
        "startup_calibration_enabled",
        default_value="true",
        description="Move to A1 and wait for /calibration_done before starting"
    )

    # ============================================================
    # Logic
    # ============================================================
    # If use_fake_hardware == false:
    #   - launch gripper adapter
    #   - joint commander uses gripper
    #
    # If use_fake_hardware == true:
    #   - do not launch gripper adapter
    #   - joint commander disables gripper
    # ============================================================

    real_robot_condition = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration("use_fake_hardware"), "' == 'false'"
        ])
    )

    commander_use_gripper = ParameterValue(
        PythonExpression([
            "'", LaunchConfiguration("use_fake_hardware"), "' == 'false'"
        ]),
        value_type=bool
    )

    # ============================================================
    # 1) UR robot driver
    # Equivalent manual command:
    # ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.102 use_fake_hardware:=false
    # ============================================================

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        }.items()
    )

    # ============================================================
    # 2) MoveIt + RViz
    # Starts after UR driver.
    # Equivalent manual command:
    # ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
    # ============================================================

    moveit = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ur_moveit_config"),
                        "launch",
                        "ur_moveit.launch.py"
                    ])
                ),
                launch_arguments={
                    "ur_type": LaunchConfiguration("ur_type"),
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                }.items()
            )
        ]
    )

    # ============================================================
    # 3) Robotiq gripper adapter
    # Launches ONLY for real robot mode.
    # Skipped when use_fake_hardware:=true.
    # Equivalent manual command:
    # ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:=192.168.1.102
    # ============================================================

    gripper_adapter = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
    condition=real_robot_condition,
    cmd=[
        'ros2',
        'run',
        'robotiq_2f_urcap_adapter',
        'robotiq_2f_adapter_node.py',
        '--ros-args',
        '-p',
        ['robot_ip:=', LaunchConfiguration("gripper_ip")]
    ],
    output='screen'
)
        ]
    )

    # ============================================================
    # 4) Joint commander
    # Always launches.
    # If use_fake_hardware:=true  -> use_gripper=false
    # If use_fake_hardware:=false -> use_gripper=true
    # ============================================================

    joint_commander = TimerAction(
        period=22.0,
        actions=[
            Node(
                package="ur5e_grip_run_cpp",
                executable="ik_commander",
                name="ee_pose_commander",
                output="screen",
                parameters=[
                    {
                        # Automatic gripper logic
                        "use_gripper": commander_use_gripper,

                        # Runtime options
                        "velocity_scale": ParameterValue(
                            LaunchConfiguration("velocity_scale"),
                            value_type=float
                        ),
                        "acceleration_scale": ParameterValue(
                            LaunchConfiguration("acceleration_scale"),
                            value_type=float
                        ),
                        "gripper_wait_sec": ParameterValue(
                            LaunchConfiguration("gripper_wait_sec"),
                            value_type=float
                        ),
                        "startup_calibration_enabled": ParameterValue(
                            LaunchConfiguration("startup_calibration_enabled"),
                            value_type=bool
                        ),

                        # Topics
                        "chess_move_topic": "/chess_move",
                        "robot_status_topic": "/robot_status",
                        "board_fen_topic": "/interface/board_fen",
                        "calibration_done_topic": "/calibration_done",

                        # MoveIt setup
                        "planning_group": "ur5e_workcell_manipulator",
                        "pose_reference_frame": "chess_board_frame",
                        "end_effector_link": "tool0",
                        "home_named_target": "up",

                        # Board setup
                        "board_size": 0.50,
                        "pick_z": 0.2,
                        "lift_z": 0.2,

                        # End-effector orientation
                        "qx": -0.010,
                        "qy": 1.000,
                        "qz": 0.002,
                        "qw": -0.015,

                        # Gripper setup
                        # These are ignored by the commander when use_gripper=false.
                        "gripper_action_name": "/robotiq_2f_urcap_adapter/gripper_command",
                        "gripper_open_position": 0.055,
                        "gripper_close_position": 0.037,
                        "gripper_start_open_position": 0.055,
                        "gripper_max_speed": 0.10,
                        "gripper_max_effort": 50.0,
                        "gripper_server_wait_sec": 5.0,

                        # Motion/path settings
                        "planning_time": 10.0,
                        "num_planning_attempts": 10,
                        "position_tolerance": 0.01,
                        "orientation_tolerance": 1.57,
                        "allow_replanning": True,

                        "bezier_points": 18,
                        "eef_step": 0.008,
                        "cartesian_fraction_min": 0.70,
                        "vertical_handle_ratio": 0.30,
                        "peak_extra_z": 0.010,

                        # Capture drop pose
                        "capture_drop_frame": "base_link",
                        "capture_drop_x": -0.162,
                        "capture_drop_y": -0.055,
                        "capture_drop_z": 1.207,
                        "capture_drop_qx": 0.999,
                        "capture_drop_qy": -0.033,
                        "capture_drop_qz": -0.023,
                        "capture_drop_qw": 0.014,
                        "capture_drop_lift_z": 0.060,

                        # Behavior
                        "return_home_after_move": True,
                        "wait_for_move_sec": 60.0,
                        "startup_delay_sec": 2.0,
                        "state_wait_sec": 10.0,
                    }
                ]
            )
        ]
    )

    return LaunchDescription([
        ur_type_arg,
        robot_ip_arg,
        gripper_ip_arg,
        use_fake_hardware_arg,
        launch_rviz_arg,
        velocity_scale_arg,
        acceleration_scale_arg,
        gripper_wait_sec_arg,
        startup_calibration_enabled_arg,

        ur_driver,
        moveit,
        gripper_adapter,
        joint_commander,
    ])
