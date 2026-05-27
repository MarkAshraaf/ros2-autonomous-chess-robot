from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    prefix = LaunchConfiguration("prefix")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=192.168.1.102",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "name:=",
            "ur5e_workcell",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=true",
            " ",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            "ur5e_workcell",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            str(moveit_config_package.perform(context)),
            "config/kinematics.yaml",
        )
    }

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            f"config/{moveit_joint_limits_file.perform(context)}",
        )
    }

    # ---------------------------------------------------------
    # Parameters for our chess IK commander node
    # ---------------------------------------------------------
    chess_commander_params = {
        # Board frame-based calibration
        "board_size": 0.50,
        "pick_z": 0.184,
        "lift_z": 0.035,
        "pose_reference_frame": "chess_board_frame",

        # Tool orientation
        "qx": -0.010,
        "qy": 1.000,
        "qz": 0.002,
        "qw": -0.015,

        # MoveIt setup
        "planning_group": "ur5e_workcell_manipulator",
        "end_effector_link": "tool0",
        "home_named_target": "up",

        "position_tolerance": 0.01,
        "orientation_tolerance": 1.57,
        "planning_time": 10.0,
        "num_planning_attempts": 10,
        "velocity_scale": 0.40,
        "acceleration_scale": 0.40,
        "allow_replanning": True,

        "startup_delay_sec": 2.0,
        "state_wait_sec": 10.0,

        # Bezier / Cartesian path settings
        "bezier_points": 18,
        "eef_step": 0.01,
        "cartesian_fraction_min": 0.60,
        "vertical_handle_ratio": 0.30,
        "peak_extra_z": 0.010,

        "return_home_after_move": True,

        # Topic input
        "chess_move_topic": "/chess_move",
        "wait_for_move_sec": 60.0,

        # Gripper settings
        "use_gripper": True,
        "gripper_action_name": "/robotiq_2f_urcap_adapter/gripper_command",

        "gripper_open_position": 0.085,
        "gripper_close_position": 0.031,
        "gripper_start_open_position": 0.085,

        "gripper_max_speed": 0.10,
        "gripper_max_effort": 50.0,

        "gripper_wait_sec": 1.0,
        "gripper_server_wait_sec": 5.0,
    }

    ik_node = Node(
        package="ur5e_grip_run_cpp",
        executable="ik_commander",
        name="ee_pose_commander",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            chess_commander_params,
        ],
    )

    return [ik_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("description_package", default_value="ur_description"),
        DeclareLaunchArgument("description_file", default_value="ur5e_workcell.urdf.xacro"),
        DeclareLaunchArgument("moveit_config_package", default_value="ur_moveit_config"),
        DeclareLaunchArgument("moveit_config_file", default_value="ur.srdf.xacro"),
        DeclareLaunchArgument("moveit_joint_limits_file", default_value="joint_limits.yaml"),
        DeclareLaunchArgument("prefix", default_value='""'),
        OpaqueFunction(function=launch_setup),
    ])