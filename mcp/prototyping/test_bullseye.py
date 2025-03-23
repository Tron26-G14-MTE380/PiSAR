import cv2
import os
import numpy as np
from bullseye_detector import BullseyeDetector
from projection import RobotHomographyProjection
from camera import CameraCalibration

# Camera Calibration
camera_calibration = CameraCalibration(
    calibration_img_width=640,
    calibration_img_height=480,
    focal_x=450,
    focal_y=450,
    center_x=320,
    center_y=240,
    skew=0
)

# Camera Position and Orientation
camera_height = 45
camera_offset = 5
camera_tilt_angle = np.deg2rad(40)
camera_axis_mapping = np.array([
    [1, 0, 0],
    [0, 0, 1],
    [0, 1, 0]
])

# Create an instance of RobotHomographyProjection
projection_instance = RobotHomographyProjection(
    camera_calibration=camera_calibration, 
    camera_position=np.array([[0], [camera_offset], [camera_height]]),
    camera_tilt_angle=camera_tilt_angle,
    axis_mapping=camera_axis_mapping
)

# Initialize the Bullseye Detector
detector = BullseyeDetector(projection_instance)

# Path to video file
video_path = os.path.join("..", "sample_videos", "lego_on_bullseye.mp4")

# Open video file
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print(f"Error: Could not open video {video_path}")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("End of video or error reading frame.")
        break

    # Optional crop like the safe zone code
    height, width, _ = frame.shape
    crop_start = height // 3
    frame = frame[crop_start:, :]

    # Bullseye Detection
    detected = detector.is_bullseye_in_scene(frame)
    center = detector.find_bullseye(frame) if detected else None

    print(f"Bullseye Detected: {detected}")
    print(f"Bullseye Center: {center}")

    if center:
        adjusted_center = (center[0], center[1] + crop_start)
        cv2.circle(frame, center, 20, (0, 255, 0), 4)
        cv2.putText(frame, "Bullseye", (center[0] - 40, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Bullseye Detection (Cropped)", frame)

    # Break if 'q' is pressed
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()