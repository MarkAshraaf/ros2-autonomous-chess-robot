#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk

import chess
import chess.engine

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


PIECE_SYMBOLS = {
    "P": "♙", "N": "♘", "B": "♗", "R": "♖", "Q": "♕", "K": "♔",
    "p": "♟", "n": "♞", "b": "♝", "r": "♜", "q": "♛", "k": "♚",
}


START_FEN = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"


# =========================================================
# GUI CLASS
# =========================================================
class ChessEngineGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Chess Robot Engine Interface")
        self.root.geometry("850x620")
        self.root.resizable(False, False)

        self.node = None
        self.square_size = 60

        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill="both", expand=True)

        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=0, column=0, padx=10, pady=10)

        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky="n", padx=10, pady=10)

        # -----------------------------
        # Chess board
        # -----------------------------
        self.canvas = tk.Canvas(
            left_frame,
            width=8 * self.square_size,
            height=8 * self.square_size,
            highlightthickness=1,
            highlightbackground="black"
        )
        self.canvas.pack()

        # -----------------------------
        # Status
        # -----------------------------
        ttk.Label(right_frame, text="Game Status", font=("Arial", 14, "bold")).pack(anchor="w")
        self.status_var = tk.StringVar(value="Starting...")
        ttk.Label(
            right_frame,
            textvariable=self.status_var,
            wraplength=320,
            justify="left"
        ).pack(anchor="w", pady=(5, 15))

        # -----------------------------
        # Human move
        # -----------------------------
        ttk.Label(right_frame, text="Last Human Move", font=("Arial", 12, "bold")).pack(anchor="w")
        self.human_move_var = tk.StringVar(value="-")
        ttk.Label(right_frame, textvariable=self.human_move_var).pack(anchor="w", pady=(5, 15))

        # -----------------------------
        # Engine move
        # -----------------------------
        ttk.Label(right_frame, text="Last Engine Move", font=("Arial", 12, "bold")).pack(anchor="w")
        self.engine_move_var = tk.StringVar(value="-")
        ttk.Label(right_frame, textvariable=self.engine_move_var).pack(anchor="w", pady=(5, 15))

        # -----------------------------
        # Manual testing input
        # -----------------------------
        ttk.Label(right_frame, text="Manual Move Test", font=("Arial", 12, "bold")).pack(anchor="w")
        ttk.Label(right_frame, text="Example: e2e4, g1f3, e7e8q").pack(anchor="w")

        manual_frame = ttk.Frame(right_frame)
        manual_frame.pack(anchor="w", pady=(5, 15))

        self.manual_move_entry = ttk.Entry(manual_frame, width=12)
        self.manual_move_entry.grid(row=0, column=0, padx=(0, 5))

        self.send_button = ttk.Button(
            manual_frame,
            text="Send",
            command=self.send_manual_move
        )
        self.send_button.grid(row=0, column=1)

        # -----------------------------
        # Move history
        # -----------------------------
        ttk.Label(right_frame, text="Move History", font=("Arial", 12, "bold")).pack(anchor="w")
        self.history_text = tk.Text(
            right_frame,
            width=40,
            height=12,
            wrap="word"
        )
        self.history_text.pack(anchor="w", pady=(5, 15))
        self.history_text.config(state="disabled")

        # -----------------------------
        # FEN
        # -----------------------------
        ttk.Label(right_frame, text="Current FEN", font=("Arial", 12, "bold")).pack(anchor="w")
        self.fen_text = tk.Text(
            right_frame,
            width=40,
            height=4,
            wrap="word"
        )
        self.fen_text.pack(anchor="w")
        self.fen_text.config(state="disabled")

        self.draw_board(START_FEN)

    def set_node(self, node):
        self.node = node

    # -------------------------------------------------
    # Manual move publisher
    # -------------------------------------------------
    def send_manual_move(self):
        move_text = self.manual_move_entry.get().strip().lower()

        if not move_text:
            return

        if self.node is not None:
            self.node.publish_detected_move(move_text)
            self.manual_move_entry.delete(0, tk.END)

    # -------------------------------------------------
    # GUI update functions
    # -------------------------------------------------
    def update_board_fen(self, fen):
        self.draw_board(fen)

        self.fen_text.config(state="normal")
        self.fen_text.delete("1.0", tk.END)
        self.fen_text.insert(tk.END, fen)
        self.fen_text.config(state="disabled")

    def update_status(self, status):
        self.status_var.set(status)

    def update_human_move(self, move):
        self.human_move_var.set(move)

    def update_engine_move(self, move):
        self.engine_move_var.set(move)

    def update_move_history(self, history):
        self.history_text.config(state="normal")
        self.history_text.delete("1.0", tk.END)
        self.history_text.insert(tk.END, history)
        self.history_text.config(state="disabled")

    # -------------------------------------------------
    # Draw board from FEN
    # -------------------------------------------------
    def draw_board(self, fen):
        self.canvas.delete("all")

        position = fen.split()[0]
        rows = position.split("/")

        # Draw squares
        for row in range(8):
            for col in range(8):
                x1 = col * self.square_size
                y1 = row * self.square_size
                x2 = x1 + self.square_size
                y2 = y1 + self.square_size

                color = "#F0D9B5" if (row + col) % 2 == 0 else "#B58863"

                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color,
                    outline=color
                )

        # Draw pieces
        for row_index, fen_row in enumerate(rows):
            col_index = 0

            for char in fen_row:
                if char.isdigit():
                    col_index += int(char)
                else:
                    piece = PIECE_SYMBOLS.get(char, "")

                    x = col_index * self.square_size + self.square_size / 2
                    y = row_index * self.square_size + self.square_size / 2

                    self.canvas.create_text(
                        x,
                        y,
                        text=piece,
                        font=("Arial", 34)
                    )

                    col_index += 1

        # Draw coordinates
        files = ["a", "b", "c", "d", "e", "f", "g", "h"]
        ranks = ["8", "7", "6", "5", "4", "3", "2", "1"]

        for i in range(8):
            self.canvas.create_text(
                i * self.square_size + 8,
                8 * self.square_size - 8,
                text=files[i],
                font=("Arial", 9, "bold"),
                anchor="sw"
            )

            self.canvas.create_text(
                5,
                i * self.square_size + 5,
                text=ranks[i],
                font=("Arial", 9, "bold"),
                anchor="nw"
            )


# =========================================================
# ROS 2 ENGINE + GUI NODE
# =========================================================
class DetectedMovesEngineGuiNode(Node):
    def __init__(self, gui: ChessEngineGUI):
        super().__init__("detected_moves_engine_gui_node")

        self.gui = gui

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("stockfish_path", "/usr/games/stockfish")
        self.declare_parameter("think_time", 0.5)

        self.stockfish_path = self.get_parameter("stockfish_path").value
        self.think_time = float(self.get_parameter("think_time").value)

        # -----------------------------
        # Chess board + Stockfish
        # -----------------------------
        self.board = chess.Board()

        try:
            self.engine = chess.engine.SimpleEngine.popen_uci(self.stockfish_path)
        except Exception as e:
            self.get_logger().error(f"Could not start Stockfish: {e}")
            raise e

        # -----------------------------
        # Subscriber from vision / mapper
        # -----------------------------
        self.detected_move_sub = self.create_subscription(
            String,
            "/detected_move",
            self.detected_move_callback,
            10
        )

        # -----------------------------
        # Publisher for robot motion node
        # -----------------------------
        self.robot_move_pub = self.create_publisher(
            String,
            "/chess_move",
            10
        )

        # -----------------------------
        # Optional interface topics
        # Still useful for debugging with ros2 topic echo
        # -----------------------------
        self.human_move_pub = self.create_publisher(String, "/interface/human_move", 10)
        self.engine_move_pub = self.create_publisher(String, "/interface/engine_move", 10)
        self.board_fen_pub = self.create_publisher(String, "/interface/board_fen", 10)
        self.status_pub = self.create_publisher(String, "/interface/status", 10)
        self.move_history_pub = self.create_publisher(String, "/interface/move_history", 10)

        # -----------------------------
        # Manual move publisher
        # GUI uses this to simulate vision
        # -----------------------------
        self.manual_detected_move_pub = self.create_publisher(
            String,
            "/detected_move",
            10
        )

        self.get_logger().info("Detected Moves Engine GUI Node is ready.")
        self.get_logger().info("Listening on /detected_move")
        self.get_logger().info("Publishing robot move on /chess_move")

        self.publish_full_interface_state("Game started.")

    # -------------------------------------------------
    # Main callback: detected human move
    # -------------------------------------------------
    def detected_move_callback(self, msg: String):
        human_move_text = msg.data.strip().lower()

        self.get_logger().info(f"Detected human move received: {human_move_text}")

        if self.board.is_game_over():
            status = f"Game is already over. Result: {self.board.result()}"
            self.publish_status(status)
            self.get_logger().warn(status)
            return

        # -----------------------------
        # 1. Convert text to chess move
        # -----------------------------
        try:
            human_move = chess.Move.from_uci(human_move_text)
        except ValueError:
            status = f"Invalid move format: {human_move_text}. Expected example: e2e4"
            self.publish_status(status)
            self.get_logger().warn(status)
            return

        # -----------------------------
        # 2. Check legality
        # -----------------------------
        if human_move not in self.board.legal_moves:
            status = f"Illegal move rejected: {human_move_text}"
            self.publish_status(status)
            self.get_logger().warn(status)
            return

        # -----------------------------
        # 3. Apply human move
        # -----------------------------
        self.board.push(human_move)

        self.publish_string(self.human_move_pub, human_move_text)
        self.gui.update_human_move(human_move_text)

        self.get_logger().info(f"Human move accepted: {human_move_text}")
        self.publish_full_interface_state(f"Human move accepted: {human_move_text}")

        # If game ended after human move
        if self.board.is_game_over():
            status = f"Game over after human move. Result: {self.board.result()}"
            self.publish_full_interface_state(status)
            self.get_logger().info(status)
            return

        # -----------------------------
        # 4. Ask Stockfish for engine move
        # -----------------------------
        try:
            result = self.engine.play(
                self.board,
                chess.engine.Limit(time=self.think_time)
            )
            engine_move = result.move

        except Exception as e:
            status = f"Stockfish failed to generate move: {e}"
            self.publish_status(status)
            self.get_logger().error(status)
            return

        if engine_move is None:
            status = "Stockfish did not return a move."
            self.publish_status(status)
            self.get_logger().warn(status)
            return

        engine_move_text = engine_move.uci()

        # -----------------------------
        # 5. Apply engine move internally
        # -----------------------------
        self.board.push(engine_move)

        self.get_logger().info(f"Engine move: {engine_move_text}")

        # -----------------------------
        # 6. Publish engine move to robot
        # -----------------------------
        self.publish_string(self.robot_move_pub, engine_move_text)

        # -----------------------------
        # 7. Update GUI and interface topics
        # -----------------------------
        self.publish_string(self.engine_move_pub, engine_move_text)
        self.gui.update_engine_move(engine_move_text)

        self.publish_full_interface_state(f"Engine move: {engine_move_text}")

    # -------------------------------------------------
    # Manual test move from GUI
    # -------------------------------------------------
    def publish_detected_move(self, move_text):
        msg = String()
        msg.data = move_text
        self.manual_detected_move_pub.publish(msg)

        self.get_logger().info(f"Manual move published to /detected_move: {move_text}")

    # -------------------------------------------------
    # Interface helper functions
    # -------------------------------------------------
    def publish_full_interface_state(self, status_text: str):
        fen = self.board.fen()
        history = self.get_move_history_text()

        self.publish_string(self.board_fen_pub, fen)
        self.publish_string(self.move_history_pub, history)

        self.gui.update_board_fen(fen)
        self.gui.update_move_history(history)

        self.publish_status(status_text)

    def publish_status(self, text: str):
        full_status = f"{text} | Board status: {self.get_board_status()}"

        self.publish_string(self.status_pub, full_status)
        self.gui.update_status(full_status)

    def get_board_status(self):
        if self.board.is_checkmate():
            return "Checkmate"
        if self.board.is_stalemate():
            return "Stalemate"
        if self.board.is_insufficient_material():
            return "Draw by insufficient material"
        if self.board.is_repetition():
            return "Draw by repetition"
        if self.board.is_check():
            return "Check"
        return "Playing"

    def get_move_history_text(self):
        temp_board = chess.Board()
        moves = []

        move_stack = list(self.board.move_stack)

        for i in range(0, len(move_stack), 2):
            move_number = (i // 2) + 1

            white_move = move_stack[i]
            white_san = temp_board.san(white_move)
            temp_board.push(white_move)

            if i + 1 < len(move_stack):
                black_move = move_stack[i + 1]
                black_san = temp_board.san(black_move)
                temp_board.push(black_move)

                moves.append(f"{move_number}. {white_san} {black_san}")
            else:
                moves.append(f"{move_number}. {white_san}")

        return " ".join(moves)

    def publish_string(self, publisher, text: str):
        msg = String()
        msg.data = text
        publisher.publish(msg)

    # -------------------------------------------------
    # Shutdown
    # -------------------------------------------------
    def destroy_node(self):
        try:
            self.engine.quit()
            self.get_logger().info("Stockfish engine closed.")
        except Exception:
            pass

        super().destroy_node()


# =========================================================
# MAIN
# =========================================================
def main(args=None):
    rclpy.init(args=args)

    gui = ChessEngineGUI()
    node = DetectedMovesEngineGuiNode(gui)
    gui.set_node(node)

    def ros_spin_once():
        rclpy.spin_once(node, timeout_sec=0.0)
        gui.root.after(30, ros_spin_once)

    gui.root.after(30, ros_spin_once)

    try:
        gui.root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()