import cv2
import numpy as np

class SafeZoneDetector:
    """Detects green rectangular safe zone in an image."""

    def __init__(self):
        pass

    def is_safe_zone_in_scene(self, frame) -> bool:
        """Checks if a safe zone is present in the frame."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV range for green
        green_mask = cv2.inRange(hsv, (40, 50, 50), (80, 255, 255))

        # Find contours
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            if len(approx) == 4:  # Looking for a rectangle
                return True

        return False

    def find_safe_zone(self, frame):
        """Finds and returns the center of the safe zone rectangle."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV range for green
        green_mask = cv2.inRange(hsv, (40, 50, 50), (80, 255, 255))

        # Find contours
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)

            if len(approx) == 4:  # Looking for a rectangle
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return (cx, cy)

        return None