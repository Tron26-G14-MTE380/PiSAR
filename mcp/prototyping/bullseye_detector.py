import cv2
import numpy as np

class BullseyeDetector:
    """Detects bullseye target in an image."""

    def __init__(self):
        pass

    def is_bullseye_in_scene(self, frame) -> bool:
        """Checks if a bullseye target is present in the frame."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for bullseye colors
        blue_mask = cv2.inRange(hsv, (90, 50, 50), (130, 255, 255))
        red_mask1 = cv2.inRange(hsv, (0, 50, 50), (10, 255, 255))
        red_mask2 = cv2.inRange(hsv, (170, 50, 50), (180, 255, 255))
        white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 40, 255))

        # Combine masks for bullseye detection
        combined_mask = cv2.bitwise_or(blue_mask, red_mask1)
        combined_mask = cv2.bitwise_or(combined_mask, red_mask2)
        combined_mask = cv2.bitwise_or(combined_mask, white_mask)

        # Find contours
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        return len(contours) >= 3  # Ensure enough rings exist

    def find_bullseye(self, frame):
        """Finds and returns the center of the bullseye target with improved accuracy."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for bullseye colors
        blue_mask = cv2.inRange(hsv, (90, 50, 50), (130, 255, 255))
        red_mask1 = cv2.inRange(hsv, (0, 50, 50), (10, 255, 255))
        red_mask2 = cv2.inRange(hsv, (170, 50, 50), (180, 255, 255))
        white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 40, 255))

        # Combine masks
        combined_mask = cv2.bitwise_or(blue_mask, red_mask1)
        combined_mask = cv2.bitwise_or(combined_mask, red_mask2)
        combined_mask = cv2.bitwise_or(combined_mask, white_mask)

        # Find contours
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print("DEBUG: No contours found")
            return None

        # Sort contours by area (largest first)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        best_circle = None
        max_radius = 0
        cv2.imshow("Processed Mask", combined_mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        for cnt in contours[:5]:  # Only check the largest 5 contours
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if 30 < radius < 200:  # Adjust range based on expected bullseye size
                print(f"DEBUG: Valid circle found at ({int(x)}, {int(y)}) with radius {radius}")
                max_radius = radius
                best_circle = (int(x), int(y))

        if best_circle is None:
            print("DEBUG: No valid bullseye found")
        
        return best_circle