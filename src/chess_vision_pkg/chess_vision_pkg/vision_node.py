import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np
from ultralytics import YOLO
import os
import json
from ament_index_python.packages import get_package_share_directory

class ChessVisionNode(Node):
    def __init__(self):
        super().__init__('chess_vision_node')
        
        # --- 1. DYNAMIC SAFE FOLDER PATH ---
        base_dir = os.path.expanduser("~/.chess_robot_data")
        homography_path = os.path.join(base_dir, "homography_matrix.npy")
        
        # --- 2. STATIC PACKAGE PATHS ---
        package_share = get_package_share_directory('chess_vision_pkg')
        npz_path = os.path.join(package_share, "calibration_data", "camera_calibration.npz")
        model_path = os.path.join(package_share, "weights", "best.pt")

        self.get_logger().info("Loading matrices and YOLO model...")

        try:
            with np.load(npz_path) as calib_data:
                self.camera_matrix = calib_data['mtx']
                self.dist_coeffs = calib_data['dist']
            self.homography_matrix = np.load(homography_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load matrices: {e}. Did you run calibrate_homography?")
            exit()

        self.model = YOLO(model_path)
        self.cap = cv2.VideoCapture(1) # Change port if needed
        
        # --- HARDWARE CAMERA SETTINGS ---
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Lock autofocus to prevent pulsing/warping
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  

        cv2.namedWindow("Chess Vision Node", cv2.WINDOW_NORMAL)
        
        # --- CALCULATE CROP AREA ---
        try:
            inv_homography = np.linalg.inv(self.homography_matrix)
            board_corners_cm = np.array([
                [[-5.625, 55.625]], [[55.625, 55.625]], 
                [[55.625, -5.625]], [[-5.625, -5.625]]
            ], dtype=np.float32)
            pixel_corners = cv2.perspectiveTransform(board_corners_cm, inv_homography)
            pts = pixel_corners.reshape(-1, 2)
            
            self.crop_x_min = max(0, int(np.min(pts[:, 0])) - 40)
            self.crop_x_max = int(np.max(pts[:, 0])) + 40
            self.crop_y_min = max(0, int(np.min(pts[:, 1])) - 120)
            self.crop_y_max = int(np.max(pts[:, 1])) + 40
        except Exception:
            self.crop_x_min = self.crop_y_min = 0
            self.crop_x_max = 1920
            self.crop_y_max = 1080

        # --- 3. ROS INTERFACE SETUP ---
        self.publisher_ = self.create_publisher(String, '/detected_pieces', 10)
        self.robot_state = "idle"
        self.status_sub = self.create_subscription(String, '/robot_status', self.status_callback, 10)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.get_logger().info("System Ready. Publishing to /detected_pieces")

    def status_callback(self, msg):
        self.robot_state = msg.data.strip().lower()

    def transform_coords(self, x, y):
        pixel_point = np.array([[[x, y]]], dtype=np.float32)
        physical_point = cv2.perspectiveTransform(pixel_point, self.homography_matrix)
        return physical_point[0][0][0], physical_point[0][0][1]

    def timer_callback(self):
        success, frame = self.cap.read()
        if not success: return

        # 🔴 FIX: ALWAYS undistort the lens first, no matter what state the robot is in!
        undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        # --- SYNC CHECK ---
        if self.robot_state == "moving":
            # Show the UNDISTORTED frame so the window never warps or changes shape
            cropped_moving_view = undistorted_frame[self.crop_y_min:self.crop_y_max, self.crop_x_min:self.crop_x_max]
            cv2.imshow("Chess Vision Node", cropped_moving_view) 
            cv2.waitKey(1)
            return

        # Garbage detection filter (65% confidence)
        results = self.model(undistorted_frame, verbose=False, conf=0.50)
        boxes = results[0].boxes
        detected_pieces = []

        if boxes is not None and len(boxes) > 0:
            for i in range(len(boxes)):
                cls_id = int(boxes.cls[i].item())
                class_name = results[0].names[cls_id]
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                
                # 🔴 EXACT CENTER CALCULATION 🔴
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0 
                
                board_x, board_y = self.transform_coords(center_x, center_y)

                # Convert to meters
                board_x = board_x / 100.0
                board_y = board_y / 100.0

                # Safely trap coordinates so they never go below 0.0 or above 0.5
                board_x = max(0.0, min(board_x, 0.5))
                board_y = max(0.0, min(board_y, 0.5))

                detected_pieces.append({
                    "piece": class_name, 
                    "x": round(float(board_x), 3), 
                    "y": round(float(board_y), 3)
                })
                
                # Draw red dot directly in the center of the piece
                cv2.circle(undistorted_frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

        msg = String()
        msg.data = json.dumps(detected_pieces)
        self.publisher_.publish(msg)

        annotated_frame = results[0].plot(img=undistorted_frame)
        cropped_view = annotated_frame[self.crop_y_min:self.crop_y_max, self.crop_x_min:self.crop_x_max]
        cv2.imshow("Chess Vision Node", cropped_view)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ChessVisionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
