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

# Paths to test images
image_paths = [
    os.path.join("sample_images", "bullseye_with_lego.jpg"),
    os.path.join("sample_images", "home_made_bullseye.jpg")
]

for image_path in image_paths:
    # Load image
    frame = cv2.imread(image_path)

    if frame is None:
        print(f"Error: Could not open image {image_path}")
        continue

    # Detect bullseye
    detected = detector.is_bullseye_in_scene(frame)
    center = detector.find_bullseye(frame) if detected else None

    print(f"Processing {image_path}")
    print(f"Bullseye Detected: {detected}")
    print(f"Bullseye Center: {center}")

    # Draw detected bullseye center
    if center:
        cv2.circle(frame, (int(center[0]), int(center[1])), 20, (0, 255, 0), 4)
        cv2.putText(frame, "Bullseye", (int(center[0]) - 40, int(center[1]) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # Display the processed image
    cv2.imshow(f"Bullseye Detection - {os.path.basename(image_path)}", frame)

    # (Optional) Save the processed images with detections
    output_path = f"processed_{os.path.basename(image_path)}"
    cv2.imwrite(output_path, frame)

# Wait for a key press and close all windows
cv2.waitKey(0)
cv2.destroyAllWindows()