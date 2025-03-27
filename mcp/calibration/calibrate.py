from time import sleep
import cv2
import glob

import numpy as np

# Define checkerboard dimensions (inner corners)
CHECKERBOARD = (6, 6)
SQUARE_SIZE = 26

IMAGE_SIZE = (3280,2464)

# Define HSV color range for black chessboard squares (adjustable)
lwr = np.array([0, 0, 200])
upr = np.array([180, 255, 255])

# Prepare object points (3D points in real-world space)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

# Arrays to store object points and image points
objpoints = []  # Real-world 3D points
imgpoints = []  # 2D image points

# Load images
images = glob.glob("captured_images/chessboard/*.jpg")

for fname in images:
    img = cv2.imread(fname)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Step 1: Color Segmentation (Extract Black Areas)
    mask = cv2.inRange(hsv, lwr, upr)

    dilated = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=5)

    cv2.imshow("Input Image", cv2.resize(dilated, (640, 480)))
    cv2.waitKey(100)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(dilated, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        chessboard_corners = cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
        cv2.imshow("Detected Corners", cv2.resize(chessboard_corners, (640, 480)))
        cv2.waitKey(100)
    else:
        print(f"Couldn't find corners for {fname}")
        sleep(100)
        raise Exception(f"Couldn't find corners for {fname}")

cv2.destroyAllWindows()

print("Calibrating...")

# Perform camera calibration
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, IMAGE_SIZE, None, None)

print("Intrinsic Matrix (K):\n", K)
print("Distortion Coefficients:\n", dist)

