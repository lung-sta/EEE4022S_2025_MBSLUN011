# adapted from https://github.com/tizianofiorenzani/how_do_drones_work/tree/master

import cv2
import numpy as np
import glob
import os
import matplotlib.pyplot as plt

BOARD_WIDTH = 8 # columns
BOARD_HEIGHT = 6 # rows
SQUARE_SIZE = 0.025 # size of a square in meters

# Folders
INPUT_DIR = os.path.expanduser("~/Desktop/Work/Proj/cal_images5")
OUTPUT_DIR = os.path.expanduser("~/Desktop/Work/Proj/cal_res_images5")

os.makedirs(OUTPUT_DIR, exist_ok=True)

# prepare 3D object points for one chessboard
objp = np.zeros((BOARD_HEIGHT * BOARD_WIDTH, 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD_WIDTH, 0:BOARD_HEIGHT].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

# storage for all images
objpoints = [] # 3D points in real world
imgpoints = [] # 2D points in image plane


images = glob.glob(os.path.join(INPUT_DIR, "*.jpg")) + \
          glob.glob(os.path.join(INPUT_DIR, "*.png"))

print(f"Found {len(images)} images in {INPUT_DIR}")

h, w = 0, 0 

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # get image dimensions for plotting later
    if h == 0 or w == 0:
        h, w = gray.shape[:2]

    # find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (BOARD_WIDTH, BOARD_HEIGHT), None)

    if ret:
        # refine corner locations
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners2)

        # draw pattern and save
        cv2.drawChessboardCorners(img, (BOARD_WIDTH, BOARD_HEIGHT), corners2, ret)
        out_name = os.path.join(OUTPUT_DIR, "corners_" + os.path.basename(fname))
        cv2.imwrite(out_name, img)
    else:
        print(f"Chessboard not found in {fname}")

if len(imgpoints) > 0:
    print("\nGenerating scatter plot of all detected corner points...")

    # variables data for plotting
    all_x = []
    all_y = []

    # flatten the list of corner arrays into two single lists (all_x, all_y)
    for corners in imgpoints:
        x_coords = corners[:, 0, 0].flatten().tolist()
        y_coords = corners[:, 0, 1].flatten().tolist()
        
        all_x.extend(x_coords)
        all_y.extend(y_coords)

    # scatter plot
    plt.figure(figsize=(10, 8))
    plt.scatter(all_x, all_y, s=5, alpha=0.6, label=f'Total Points: {len(all_x)} from {len(imgpoints)} images')

    plt.xlim(0, w)
    plt.ylim(h, 0) 

    # Add titles and labels
    plt.title('Distribution of Calibration Corner Points Across Image Plane')
    plt.xlabel('X-coordinate (Pixels)')
    plt.ylabel('Y-coordinate (Pixels)')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.gca().set_aspect('equal', adjustable='box')
    
    # save
    plot_path = os.path.join(OUTPUT_DIR, "corner_distribution.png")
    plt.savefig(plot_path)
    print(f"Corner distribution graph saved to: {plot_path}")
  
# calibration
if len(objpoints) > 0:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (w, h), None, None
    )

    print("\nCalibration Results")
    print("Camera matrix:\n", mtx)
    print("Distortion coefficients:\n", dist.ravel())

    # save parameters
    np.savez(os.path.join(OUTPUT_DIR, "calibration_data.npz"),
             camera_matrix=mtx, dist_coeff=dist,
             rvecs=rvecs, tvecs=tvecs) 

    # undistort
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    for fname in images:
        img = cv2.imread(fname)
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

        x, y, w_roi, h_roi = roi
        dst = dst[y:y+h_roi, x:x+w_roi]

        out_name = os.path.join(OUTPUT_DIR, "undist_" + os.path.basename(fname))
        cv2.imwrite(out_name, dst)

    # reprojection errors
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    print("Mean reprojection error:", mean_error / len(objpoints))
else:
    print("No valid chessboard images found, calibration failed.")
