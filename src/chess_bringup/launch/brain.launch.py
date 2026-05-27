from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. The YOLO Vision Node
        # Starts instantly (t=0s) to give PyTorch/YOLO time to load the model
        Node(
            package='chess_vision_pkg',
            name='chess_vision_node',
            output='screen',
            executable='/home/mark/IR_vision/yolo_venv/bin/python3',
            arguments=['/home/mark/ir_project/install/chess_vision_pkg/lib/chess_vision_pkg/vision_node']
        ),
        
        # 2. The State Mapper Node
        # Starts at t=10s after Vision is fully online
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='chess_mapper_state',
                    executable='mapper_state_node',
                    name='mapper_state_node',
                    output='screen'
                )
            ]
        ),
        
        # 3. The Game Engine Node
        # Starts at t=15s to ensure the mapper has established the initial board state
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='chess_engine_bridge',
                    executable='detected_moves_engine_node',
                    name='detected_moves_engine_node',
                    output='screen'
                )
            ]
        )
    ])
