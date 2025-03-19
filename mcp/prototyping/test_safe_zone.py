from projection import RobotHomographyProjection
from camera import CameraCalibration
import numpy as np
from safe_zone_detector import SafeZoneDetector
import os
import cv2

# Camera Calibration Setup
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

# ✅ **Create an instance of RobotHomographyProjection**
projection_instance = RobotHomographyProjection(
    camera_calibration=camera_calibration, 
    camera_position=np.array([[0], [camera_offset], [camera_height]]),
    camera_tilt_angle=camera_tilt_angle,
    axis_mapping=camera_axis_mapping
)

# ✅ **Pass the instance to SafeZoneDetector**
detector = SafeZoneDetector(projection_instance)

# Path to video file
video_path = os.path.join("..", "sample_videos", "full_course_vid_480p.mp4")

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

    # Get video dimensions
    height, width, _ = frame.shape

    # Crop the top 33% (Keep only the bottom 67%)
    crop_start = height // 3  # Top 33% boundary
    frame = frame[crop_start:, :]  # Crop height, keep full width

    # Apply safe zone detection
    detected = detector.is_safe_zone_in_scene(frame)
    center = detector.find_safe_zone(frame) if detected else None

    print(f"Safe Zone Detected: {detected}")
    print(f"Safe Zone Center: {center}")

    # Adjust the center position since the frame is cropped
    if center:
        adjusted_center = (center[0], center[1] + crop_start)
        cv2.circle(frame, center, 10, (0, 255, 0), -1)  # Draw inside cropped frame

    # Display the cropped and processed frame
    cv2.imshow("Safe Zone Detection (Cropped)", frame)

    # Break loop if 'q' is pressed
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()