import cv2
import numpy as np

class SafeZoneDetector:
    """Detects and reconstructs a green rectangular safe zone in an image, even if partially occluded."""

    def __init__(self):
        pass

    def is_safe_zone_in_scene(self, frame) -> bool:
        """Checks if a safe zone is present in the frame."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, (40, 50, 50), (80, 255, 255))

        # Debugging visualization
        cv2.imshow("HSV Image", hsv)
        cv2.imshow("Green Mask", green_mask)
        cv2.waitKey(1)

        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"Found {len(contours)} green contours")

        return len(contours) > 0  # If there are any green contours, assume a safe zone is present

    def find_safe_zone(self, frame):
        """Finds the safe zone and computes its center even if it's partially occluded."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, (40, 50, 50), (80, 255, 255))

        # Show initial mask
        cv2.imshow("Green Mask", green_mask)
        cv2.waitKey(1)

        # Find contours
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"Contours found: {len(contours)}")

        if len(contours) == 0:
            print("No safe zone detected.")
            return None

        # Merge contours using convex hull
        all_points = np.vstack([c.reshape(-1, 2) for c in contours])
        hull = cv2.convexHull(all_points)

        # Draw hull for debugging
        debug_image = frame.copy()
        cv2.drawContours(debug_image, [hull], -1, (255, 255, 255), 2)
        cv2.imshow("Convex Hull Shape", debug_image)
        cv2.waitKey(1)