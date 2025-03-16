import cv2
import os
import numpy as np
from bullseye_detector import BullseyeDetector

# Initialize detector
detector = BullseyeDetector()

# Path to hardcoded test image
image_path = os.path.join("..", "sample_images", "bullseye.jpg")

# Load image
img = cv2.imread(image_path)
if img is None:
    print(f"Error: Could not load {image_path}")
else:
    height, width, _ = img.shape  # Get image dimensions

    # Detect bullseye
    detected = detector.is_bullseye_in_scene(img)
    center = detector.find_bullseye(img) if detected else None

    print(f"Bullseye Detected: {detected}")
    print(f"Bullseye Center: {center}")

    # Draw bullseye detection if found
    if center:
        print(f"DEBUG: Drawing circle at {center}")

        # Draw detected bullseye center
        cv2.circle(img, center, 20, (0, 255, 0), 4)  
        cv2.putText(img, "Bullseye", (center[0] - 40, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # Show the processed image with detection overlay
    cv2.imshow("Bullseye Detection with Axes", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()