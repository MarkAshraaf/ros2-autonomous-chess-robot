import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

def main():
    # --- 1. DYNAMIC SAFE FOLDER (Where we SAVE the Homography) ---
    safe_dir = os.path.expanduser("~/.chess_robot_data")
    os.makedirs(safe_dir, exist_ok=True)
    save_path = os.path.join(safe_dir, "homography_matrix.npy")

    # --- 2. STATIC PACKAGE FOLDER (Where we LOAD the Intrinsics) ---
    try:
        package_share = get_package_share_directory('chess_vision_pkg')
        calib_file = os.path.join(package_share, "calibration_data", "camera_calibration.npz")
        calib_data = np.load(calib_file)
        mtx, dist = calib_data['mtx'], calib_data['dist']
    except Exception as e:
        print(f"Error loading static camera intrinsics from package: {e}")
        return

    # --- ARUCO SETUP ---
    CAMERA_PORT = 1 # Change to 0, 1, or 2 depending on your laptop
    TARGET_IDS = [46, 37, 32, 8]
    TARGET_COORDS = np.array([
        [-5.625, 55.625], [55.625, 55.625], [55.625, -5.625], [-5.625, -5.625]
    ], dtype=np.float32)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    cap = cv2.VideoCapture(CAMERA_PORT)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    print("\nPress 'SPACE' to SAVE Homography Matrix when green box appears. 'q' to QUIT.")

    while True:
        ret, frame = cap.read()
        if not ret: break

        frame_undist = cv2.undistort(frame, mtx, dist, None, mtx)
        corners, ids, rejected = detector.detectMarkers(frame_undist)
        ready_to_save, current_src_pts = False, None

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame_undist, corners, ids)
            ids_list = ids.flatten().tolist()
            
            if all(m_id in ids_list for m_id in TARGET_IDS):
                src_pts = []
                for m_id in TARGET_IDS:
                    idx = ids_list.index(m_id)
                    center_x, center_y = np.mean(corners[idx][0], axis=0)
                    src_pts.append([center_x, center_y])
                    
                current_src_pts = np.array(src_pts, dtype=np.float32)
                ready_to_save = True 
                cv2.polylines(frame_undist, [np.int32(current_src_pts)], True, (0, 255, 0), 3)
                
        display_frame = cv2.resize(frame_undist, (1280, 720))
        cv2.imshow("ArUco Calibration", display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):  
            if ready_to_save:
                H, status = cv2.findHomography(current_src_pts, TARGET_COORDS)
                np.save(save_path, H)
                print(f"\nSUCCESS! Homography Matrix Saved to '{save_path}'")
                break
            else:
                print("Cannot save! Ensure all 4 markers are visible.")
        elif key == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
