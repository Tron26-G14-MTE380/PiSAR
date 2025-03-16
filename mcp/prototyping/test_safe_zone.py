import cv2
import os
from safe_zone_detector import SafeZoneDetector

# Initialize detector
detector = SafeZoneDetector()

# Path to hardcoded test image
image_path = os.path.join("..", "sample_images", "safe_zone.jpg")

# Load image
img = cv2.imread(image_path)
if img is None:
    print(f"Error: Could not load {image_path}")
else:
    detected = detector.is_safe_zone_in_scene(img)
    center = detector.find_safe_zone(img) if detected else None

    print(f"Safe Zone Detected: {detected}")
    print(f"Safe Zone Center: {center}")

    if center:
        cv2.circle(img, center, 10, (0, 255, 0), -1)
        cv2.imshow("Safe Zone Detection", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()