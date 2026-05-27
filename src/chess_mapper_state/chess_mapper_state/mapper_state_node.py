#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time


# =========================================================
# Configuration
# =========================================================

# Same values from your old working code
SQUARE_SIZE = 0.063
MARGIN = 0.003

MIN_MOVE_DISTANCE = 0.025
PROCESS_INTERVAL = 1.0

FILES = "abcdefgh"

ALL_SQUARES = [
    f"{file}{rank}"
    for rank in range(1, 9)
    for file in FILES
]


class MapperStateNode(Node):
    def __init__(self):
        super().__init__("mapper_state_node")

        # Last accepted camera board
        self.previous_board = None

        # Store previous real positions for noise filtering
        self.previous_positions = {}

        # Time filter
        self.last_process_time = 0.0

        # Robot status
        self.robot_is_moving = False

        # Robot move tracking
        self.latest_robot_move = None
        self.expected_board_after_robot = None

        # After robot becomes idle, we check the next camera frame
        self.need_post_robot_check = False

        # Avoid getting stuck forever
        self.post_robot_uncertain_count = 0
        self.max_post_robot_uncertain_frames = 5

        # =====================================================
        # Subscribers
        # =====================================================

        self.sub = self.create_subscription(
            String,
            "/detected_pieces",
            self.callback,
            10
        )

        self.robot_sub = self.create_subscription(
            String,
            "/robot_status",
            self.robot_callback,
            10
        )

        self.chess_move_sub = self.create_subscription(
            String,
            "/chess_move",
            self.chess_move_callback,
            10
        )

        # =====================================================
        # Publishers
        # =====================================================

        self.board_pub = self.create_publisher(
            String,
            "/board_state_raw",
            10
        )

        self.move_pub = self.create_publisher(
            String,
            "/detected_move",
            10
        )

        self.get_logger().info("Mapper + State Node Started")
        self.get_logger().info("Subscribed to /detected_pieces")
        self.get_logger().info("Subscribed to /robot_status")
        self.get_logger().info("Subscribed to /chess_move")

    # =========================================================
    # Normalize piece labels
    # =========================================================

    def normalize_piece_name(self, piece):
        if piece is None:
            return None

        return str(piece).strip().lower().replace(" ", "_").replace("-", "_")

    # =========================================================
    # Robot status callback
    # =========================================================

    def robot_callback(self, msg):
        status = msg.data.strip().lower()

        if status == "moving":
            self.robot_is_moving = True

            self.get_logger().info(
                "Robot moving → camera frames will be stored but move detection is delayed"
            )

        elif status == "idle":
            self.robot_is_moving = False

            if self.expected_board_after_robot is not None:
                self.need_post_robot_check = True
                self.post_robot_uncertain_count = 0

                self.get_logger().info(
                    "Robot idle → next camera frame will be compared with expected board after robot move"
                )
            else:
                self.get_logger().info("Robot idle → no expected robot board, normal mode")

        elif status == "calibrating":
            self.robot_is_moving = True
            self.need_post_robot_check = False
            self.expected_board_after_robot = None
            self.latest_robot_move = None

            self.get_logger().info("Robot calibrating → move detection delayed")

        else:
            self.get_logger().warning(f"Unknown robot status received: {status}")

    # =========================================================
    # Chess move callback
    # =========================================================

    def chess_move_callback(self, msg):
        move = msg.data.strip().lower()

        if len(move) < 4:
            self.get_logger().warning(f"Invalid /chess_move received: {move}")
            return

        move = move[:4]
        self.latest_robot_move = move

        self.get_logger().info(f"Robot/engine move received on /chess_move: {move}")

        if self.previous_board is None:
            self.get_logger().warning(
                "Cannot prepare expected board after robot move because previous_board is None"
            )
            self.expected_board_after_robot = None
            return

        expected_board, ok = self.apply_robot_move_to_board(
            self.previous_board,
            move
        )

        if not ok:
            self.get_logger().warning(
                f"Could not apply robot move {move} to previous camera board. "
                "Post-robot delayed detection may be skipped."
            )
            self.expected_board_after_robot = None
            return

        self.expected_board_after_robot = expected_board

        self.get_logger().info(
            f"Expected board after robot move {move}: {self.expected_board_after_robot}"
        )

    # =========================================================
    # Apply robot move internally
    # =========================================================

    def apply_robot_move_to_board(self, board, move):
        new_board = board.copy()

        source = move[0:2]
        destination = move[2:4]

        moving_piece = new_board.get(source)

        if moving_piece is None:
            return new_board, False

        # Remove source square
        new_board.pop(source, None)

        # Put robot piece at destination
        # If destination had an opponent piece, this overwrites it.
        new_board[destination] = moving_piece

        return new_board, True

    # =========================================================
    # Coordinate mapping
    # =========================================================

    def xy_to_square(self, x, y):
        col = int(math.floor(x / SQUARE_SIZE))
        row = int(math.floor(y / SQUARE_SIZE))

        if col < 0 or col > 7 or row < 0 or row > 7:
            return None

        local_x = x - col * SQUARE_SIZE
        local_y = y - row * SQUARE_SIZE

        if (
            local_x < MARGIN or local_x > SQUARE_SIZE - MARGIN or
            local_y < MARGIN or local_y > SQUARE_SIZE - MARGIN
        ):
            return None

        return FILES[col] + str(row + 1)

    # =========================================================
    # Jitter filtering
    # =========================================================

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def filter_small_jitter(self, detections):
        filtered = []

        for det in detections:
            try:
                piece = self.normalize_piece_name(det["piece"])
                x = float(det["x"])
                y = float(det["y"])
            except KeyError:
                self.get_logger().warning(f"Invalid detection format: {det}")
                continue

            if piece in self.previous_positions:
                old_x, old_y = self.previous_positions[piece]

                dist = self.distance(x, y, old_x, old_y)

                if dist < MIN_MOVE_DISTANCE:
                    x = old_x
                    y = old_y

            filtered.append({
                "piece": piece,
                "x": x,
                "y": y
            })

        return filtered

    def update_previous_positions(self, detections):
        self.previous_positions = {
            self.normalize_piece_name(det["piece"]): (float(det["x"]), float(det["y"]))
            for det in detections
            if "piece" in det and "x" in det and "y" in det
        }

    # =========================================================
    # Build board from detections
    # =========================================================

    def build_board_from_detections(self, detections):
        board = {}

        for det in detections:
            try:
                piece = self.normalize_piece_name(det["piece"])
                x = float(det["x"])
                y = float(det["y"])
            except KeyError:
                self.get_logger().warning(f"Invalid detection format: {det}")
                continue

            square = self.xy_to_square(x, y)

            if square is None:
                self.get_logger().warning(
                    f"Ignored {piece}: x={x:.3f}, y={y:.3f}"
                )
                continue

            board[square] = piece

        return board

    # =========================================================
    # State-change detection
    # =========================================================

    def detect_state_change_between(self, old_board, current_board):
        changed_squares = []

        for square in ALL_SQUARES:
            old_piece = self.normalize_piece_name(old_board.get(square))
            new_piece = self.normalize_piece_name(current_board.get(square))

            if old_piece != new_piece:
                changed_squares.append({
                    "square": square,
                    "old": old_piece,
                    "new": new_piece
                })

        if len(changed_squares) == 0:
            return None, "no_change"

        source = None
        destination = None
        moving_piece = None

        # Source: square where a piece disappeared
        for change in changed_squares:
            if change["old"] is not None and change["new"] is None:
                source = change["square"]
                moving_piece = change["old"]
                break

        # Destination: square where same piece appeared
        for change in changed_squares:
            if change["new"] == moving_piece:
                destination = change["square"]
                break

        if source is not None and destination is not None:
            move = source + destination
            return move, "valid_move"

        self.get_logger().info(f"Changed squares count: {len(changed_squares)}")
        self.get_logger().info(f"Changed squares: {changed_squares}")

        return None, "uncertain"

    def detect_state_change(self, current_board):
        if self.previous_board is None:
            self.previous_board = current_board.copy()
            return None, "initialized"

        move, status = self.detect_state_change_between(
            self.previous_board,
            current_board
        )

        if status == "valid_move":
            self.previous_board = current_board.copy()

        return move, status

    # =========================================================
    # Publish helper
    # =========================================================

    def publish_detected_move(self, move):
        msg = String()
        msg.data = move
        self.move_pub.publish(msg)

    # =========================================================
    # Clear robot-post-check state
    # =========================================================

    def clear_post_robot_state(self):
        self.need_post_robot_check = False
        self.expected_board_after_robot = None
        self.latest_robot_move = None
        self.post_robot_uncertain_count = 0

    # =========================================================
    # Main camera callback
    # =========================================================

    def callback(self, msg):
        now = time.time()

        if now - self.last_process_time < PROCESS_INTERVAL:
            return

        self.last_process_time = now

        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON received")
            return

        if not isinstance(detections, list):
            self.get_logger().error("Invalid /detected_pieces format: expected JSON list")
            return

        filtered_detections = self.filter_small_jitter(detections)
        current_board = self.build_board_from_detections(filtered_detections)

        board_msg = String()
        board_msg.data = json.dumps(current_board)
        self.board_pub.publish(board_msg)

        # =====================================================
        # Case 1: robot is moving
        # Store board, but do not detect/publish human move now
        # =====================================================
        if self.robot_is_moving:
            self.update_previous_positions(filtered_detections)

            self.get_logger().info(
                f"Robot moving → camera board stored only: {current_board}"
            )
            return

        # =====================================================
        # Case 2: robot just finished
        # Compare expected board after robot move with camera board
        # =====================================================
        if self.need_post_robot_check:
            if self.expected_board_after_robot is None:
                self.get_logger().warning(
                    "Post-robot check requested but expected_board_after_robot is None. "
                    "Syncing to current camera board."
                )

                self.previous_board = current_board.copy()
                self.update_previous_positions(filtered_detections)
                self.clear_post_robot_state()
                return

            move, status = self.detect_state_change_between(
                self.expected_board_after_robot,
                current_board
            )

            self.get_logger().info(
                f"Post-robot expected board: {self.expected_board_after_robot}"
            )
            self.get_logger().info(
                f"Post-robot camera board: {current_board}"
            )
            self.get_logger().info(
                f"Post-robot check status: {status}"
            )

            if status == "valid_move":
                self.publish_detected_move(move)

                self.get_logger().info(
                    f"Delayed human move detected after robot move: {move}"
                )

                self.previous_board = current_board.copy()
                self.update_previous_positions(filtered_detections)
                self.clear_post_robot_state()
                return

            if status == "no_change":
                self.get_logger().info(
                    "No delayed human move detected after robot move. Syncing board."
                )

                self.previous_board = current_board.copy()
                self.update_previous_positions(filtered_detections)
                self.clear_post_robot_state()
                return

            # uncertain
            self.post_robot_uncertain_count += 1

            self.get_logger().warning(
                f"Post-robot comparison uncertain "
                f"({self.post_robot_uncertain_count}/{self.max_post_robot_uncertain_frames})"
            )

            if self.post_robot_uncertain_count >= self.max_post_robot_uncertain_frames:
                self.get_logger().warning(
                    "Too many uncertain post-robot frames. "
                    "Forcing sync so mapper does not get stuck."
                )

                self.previous_board = current_board.copy()
                self.update_previous_positions(filtered_detections)
                self.clear_post_robot_state()
                return

            self.update_previous_positions(filtered_detections)
            return

        # =====================================================
        # Case 3: normal human move detection
        # This is basically your old working behavior
        # =====================================================
        move, status = self.detect_state_change(current_board)

        self.update_previous_positions(filtered_detections)

        self.get_logger().info(f"Board: {current_board}")
        self.get_logger().info(f"Status: {status}")

        if status == "valid_move":
            self.publish_detected_move(move)
            self.get_logger().info(f"Detected human move: {move}")


def main(args=None):
    rclpy.init(args=args)

    node = MapperStateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()