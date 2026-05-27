# Autonomous Chess Robot Using Computer Vision, Stockfish, and ROS2

This project implements an autonomous chess-playing robot pipeline that integrates **computer vision**, **chess logic**, **Stockfish**, **ROS2**, **MoveIt**, and a **UR5e robot with a Robotiq 2F gripper**.

The system detects the human move from a camera view of the chessboard, updates a chess interface, asks Stockfish for the best response move, and sends that move to the robot commander for execution.

---

## Project Overview

The system follows this workflow:

```text
Camera Calibration using 4 ArUco markers
→ YOLOv11 Chess Piece Detection
→ Mapper: x,y positions to chess squares
→ State-Change Detection: detect human move
→ Chess Interface: update GUI and publish FEN
→ Stockfish Game Engine: calculate best move
→ Robot Commander: convert chess move to robot poses
→ MoveIt Motion Planning
→ Robotiq Gripper Execution
```

Example detected human move:

```text
e2e4
```

Example robot move from Stockfish:

```text
a2a4
```

---

## Main Features

- Camera-to-board calibration using four ArUco markers.
- YOLOv11-based chess piece detection.
- Real-world coordinate mapping from image detections to chessboard positions.
- Mapper node that converts `(x, y)` positions into chess squares such as `e2`.
- State-change detection to identify human moves.
- Chess GUI/interface with FEN publishing.
- Stockfish integration for best-move calculation.
- UR5e robot command pipeline using MoveIt.
- Robotiq 2F gripper adapter integration.
- Capture handling using FEN board-state checking.
- Robot-status synchronization to prevent false vision updates during robot motion.

---

## Hardware Setup

The physical setup includes:

- UR5e robot arm.
- Robotiq 2F gripper.
- Top-mounted camera above the chessboard.
- 50 cm × 50 cm chessboard.
- 6.25 cm × 6.25 cm square size.
- Four ArUco markers placed around the chessboard for camera-to-board calibration.
- Wooden chess pieces with approximate dimensions of 3 cm × 3 cm × 6 cm.

The chessboard frame is defined as:

```text
Origin: bottom-left corner of A1
+x direction: a-file to h-file
+y direction: rank 1 to rank 8
```

---

## Vision System

The vision pipeline starts by calibrating the camera using four ArUco markers. YOLO then detects each visible chess piece and publishes the detected piece label with its calculated position in the chessboard frame.

The YOLO detection output is published on:

```text
/detected_pieces
```

Message type:

```text
std_msgs/String
```

Example output:

```json
[
  {"piece": "white_pawn", "x": 0.03125, "y": 0.09375},
  {"piece": "black_king", "x": 0.28125, "y": 0.40625}
]
```

The detection model was trained using **YOLOv11** on a Roboflow dataset. A confidence threshold of **0.60** was used to reduce false detections.

Dataset summary:

| Split | Images |
|---|---:|
| Train | 1044 |
| Validation | 76 |
| Test | 38 |
| Total | 1158 |

Image size:

```text
640 × 640
```

Augmentations included flipping, rotation, exposure adjustment, and slight blur.

---

## Mapper and State-Change Detection

The mapper converts real-world `(x, y)` positions into chess squares.

Square size:

```text
S = 0.0625 m
```

Mapping equations:

```text
col = floor(x / S)
row = floor(y / S)
```

Example:

```text
x = 0.281 m
y = 0.090 m
```

Then:

```text
col = floor(0.281 / 0.0625) = 4 → e
row = floor(0.090 / 0.0625) = 1 → rank 2
```

So the mapped square is:

```text
e2
```

The state-change node compares the previous and current board state to detect a move.

Example:

```text
Before: white_pawn at e2
After:  white_pawn at e4
Detected move: e2e4
```

---

## Chess Interface and Stockfish

The chess interface receives the detected human move from:

```text
/detected_move
```

It updates the GUI and publishes the current board FEN on:

```text
/interface/board_fen
```

The Stockfish engine receives the FEN and calculates the best move. The selected move is published to:

```text
/chess_move
```

This topic is subscribed by the robot commander.

---

## Robot System

The main robot commander node is:

```text
ee_pose_commander
```

Subscribed topics:

```text
/chess_move
/interface/board_fen
/calibration_done
```

Published topic:

```text
/robot_status
```

Robot status values:

```text
moving
idle
```

The robot converts chess moves, such as:

```text
a2a4
```

into Cartesian poses in the `chess_board_frame`.

The normal pick-and-place sequence is:

```text
Move above source square
Move down to source square
Close gripper
Lift piece
Move above target square
Move down to target square
Open gripper
Move back above target square
Return home if enabled
```

---

## Capture Handling

Before executing a move, the robot commander checks the latest FEN from:

```text
/interface/board_fen
```

If the target square is occupied, the robot first removes the captured piece and places it outside the board.

Capture sequence:

```text
Move to occupied target square
Pick captured piece
Move to capture drop pose
Release captured piece outside the board
Return home
Execute normal chess move
```

Captured-piece drop pose is expressed in:

```text
base_link
```

Example drop pose:

```yaml
capture_drop_x: -0.162
capture_drop_y: -0.055
capture_drop_z: 1.207
```

---

## Important ROS2 Topics

| Topic | Description |
|---|---|
| `/detected_pieces` | YOLO output containing piece labels and x,y positions |
| `/board_state_raw` | Current board state after mapping |
| `/detected_move` | Human move detected by state-change logic |
| `/interface/board_fen` | FEN board state published by the chess interface |
| `/chess_move` | Best move sent from Stockfish to the robot commander |
| `/calibration_done` | Startup calibration confirmation topic |
| `/robot_status` | Robot status used for synchronization |

---

## Software Dependencies

| Package | Purpose |
|---|---|
| ROS2 | Node communication and system integration |
| OpenCV | Image processing and ArUco detection |
| NumPy | Matrix and coordinate operations |
| Ultralytics YOLO | Chess piece detection |
| python-chess | Legal move handling and FEN generation |
| Stockfish | Chess engine |
| MoveIt2 | Robot motion planning |
| RViz | Robot and chessboard visualization |
| Robotiq 2F URCap Adapter | Gripper control |

Example installation commands:

```bash
pip install opencv-python opencv-contrib-python
pip install numpy
pip install ultralytics
pip install chess
sudo apt install stockfish
```

---

## Robot Configuration Example

```yaml
robot:
  name: "UR5e"
  planning_group: "ur5e_workcell_manipulator"
  base_frame: "base_link"
  pose_reference_frame: "chess_board_frame"
  end_effector_link: "tool0"
  home_named_target: "up"

chessboard:
  board_size: 0.50
  square_size: 0.0625
  frame: "chess_board_frame"
  origin: "bottom-left corner of A1"

motion:
  pick_z: 0.2
  lift_z: 0.2
  velocity_scale: 0.40
  acceleration_scale: 0.40
  planning_time: 10.0
  num_planning_attempts: 10
  return_home_after_move: true

topics:
  detected_pieces: "/detected_pieces"
  board_state_raw: "/board_state_raw"
  detected_move: "/detected_move"
  board_fen: "/interface/board_fen"
  chess_move: "/chess_move"
  robot_status: "/robot_status"
  calibration_done: "/calibration_done"

capture:
  drop_frame: "base_link"
  drop_x: -0.162
  drop_y: -0.055
  drop_z: 1.207

gripper:
  use_gripper: true
  action_name: "/robotiq_2f_urcap_adapter/gripper_command"
  open_position: 0.050
  close_position: 0.037
  max_speed: 0.10
  max_effort: 50.0
```

---

## Running the System

Recommended running order:

1. Launch the UR robot driver.
2. Launch MoveIt.
3. Run the Robotiq gripper adapter.
4. Run the robot commander.
5. Wait until the robot reaches the A1 calibration pose.
6. Confirm startup calibration using `/calibration_done`.
7. Run camera calibration or load the saved calibration.
8. Run the YOLO detection node.
9. Run the mapper and state-change node.
10. Run the chess interface with Stockfish engine node.

### Launch Robot Driver

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.102 use_fake_hardware:=true
```

### Launch MoveIt

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e robot_ip:=192.168.1.102 use_fake_hardware:=true
```

### Run Robotiq Gripper Adapter

```bash
ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:=192.168.1.102
```

### Run Robot Commander

```bash
ros2 launch ur5e_grip_run_cpp ik_commander.launch.py
```

### Confirm Startup Calibration

```bash
ros2 topic pub --once /calibration_done std_msgs/msg/String "{data: 'done'}"
```

### Run Vision and Chess Nodes

```bash
# Run camera calibration or camera node
ros2 run chess_vision_pkg calibrate_homography

# Run YOLO detection node
ros2 run chess_vision_pkg vision_node

# Run mapper and state-change node
ros2 run chess_mapper_state mapper_state_node

# Run chess interface with Stockfish engine node
ros2 run chess_engine_bridge detected_moves_engine_node
```

---

## Testing

### Test Mapper and State-Change Node

Publish initial board:

```bash
ros2 topic pub --once /detected_pieces std_msgs/msg/String "data: '[{\"piece\":\"white_pawn\",\"x\":0.03125,\"y\":0.09375}]'"
```

Publish moved board:

```bash
ros2 topic pub --once /detected_pieces std_msgs/msg/String "data: '[{\"piece\":\"white_pawn\",\"x\":0.03125,\"y\":0.15625}]'"
```

Expected output:

```text
/detected_move = a2a3
```

### Test Robot Move

```bash
ros2 topic pub --once /chess_move std_msgs/msg/String "{data: 'e2e4'}"
```

### Echo Robot Status

```bash
ros2 topic echo /robot_status
```

### Echo Board FEN

```bash
ros2 topic echo /interface/board_fen
```

---

## Integration Challenges Solved

### Unit Mismatch

The vision node initially produced coordinates in centimeters, while the mapper expected meters. This was fixed by converting the values before publishing:

```python
x_m = x_cm / 100.0
y_m = y_cm / 100.0
```

### Out-of-Bounds Coordinates

Sometimes the vision node produced values slightly outside the 0.50 m board range. Boundary clamping was added:

```python
BOARD_SIZE = 0.50
EPSILON = 1e-6

x = min(max(x, 0.0), BOARD_SIZE - EPSILON)
y = min(max(y, 0.0), BOARD_SIZE - EPSILON)
```

### Robot Synchronization

During robot motion, the mapper uses `/robot_status` to avoid detecting the robot movement as a human move.

### Capture Handling

The robot checks `/interface/board_fen` before executing a move to decide whether the target square is occupied.

---

## Limitations

- YOLO may confuse piece types or colors.
- Pieces near square borders may be ignored due to margin filtering.
- The system still requires tuning for margin, minimum movement distance, and processing interval.
- Castling, promotion, and en passant require additional handling.
- Manual robot-to-board calibration must be repeated if the setup changes.

---

## Future Work

- Improve YOLO dataset and detection accuracy.
- Add confidence filtering and majority voting over multiple frames.
- Improve automatic robot-to-workpiece calibration.
- Publish transforms using ROS2 TF2.
- Add collision objects for pieces in the motion planner.
- Add full support for castling, promotion, and en passant.
- Improve the GUI to show detected moves, engine moves, robot status, and capture status.

---

## Team Members

| Name |
|---|
| Yusr Mohamed |
| Rewan Hagag |
| Rana Mohamed |
| Noor Khaled |
| Salma Wael |
| Jana Ashraf |
| Fady Hani |
| Mark Ashraf |
| Reham Hassan |
| Omar Hindawi |

---

## Documentation

The full technical report includes the detailed system description, figures, node architecture, configuration, tests, and integration challenges.

Recommended repository structure:

```text
.
├── README.md
├── docs/
│   └── IR_project_report.pdf
├── images/
│   ├── hardware_setup.jpeg
│   ├── chess_with_aruco.jpeg
│   ├── yolo_website.jpeg
│   ├── yolo_with_xy_positions.jpeg
│   ├── state_change_mapper_terminal.jpeg
│   ├── interface.jpeg
│   └── robot_setup_rviz.jpeg
├── src/
├── config/
└── models/
```

---

## Notes

This README summarizes the project for GitHub. For full details, refer to the project report in the `docs/` folder.
