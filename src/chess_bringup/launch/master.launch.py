from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. Expose the fake hardware argument at the master level
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false", 
        description="true for fake/simulation hardware, false for real robot"
    )

    bringup_pkg_share = FindPackageShare('chess_bringup')

    # 2. Include the Brains (Starts at t=0s)
    # Inside this file, Vision starts at 0s, Mapper at 10s, Engine at 15s
    brain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_pkg_share, 'launch', 'brain.launch.py'])
        )
    )

    # 3. Include the Robot Hardware (Starts at t=20s)
    # We hold the entire robot launch file back for 20 seconds to give the AI time to boot
    robot_launch = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_pkg_share, 'launch', 'robot.launch.py'])
                ),
                # Pass the master argument down to the robot launch file
                launch_arguments={
                    'use_fake_hardware': LaunchConfiguration('use_fake_hardware')
                }.items()
            )
        ]
    )

    return LaunchDescription([
        use_fake_hardware_arg,
        brain_launch,
        robot_launch
    ])
