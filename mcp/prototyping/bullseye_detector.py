import cv2
import numpy as np
from projection import RobotHomographyProjection


class BullseyeDetector:
    """Detects bullseye target in an image."""

    def __init__(self, projection:RobotHomographyProjection):
        self.projection = projection
        pass

    def is_bullseye_in_scene(self, frame) -> bool:
        """Checks if a bullseye target is present in the frame by checking if `find_bullseye()` returns a valid center."""
        center = self.find_bullseye(frame)
        return center is not None  # Only return True if we actually found a bullseye center

    def _hsv_filter(self, frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, (0, 0, 220), (180, 40, 255))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

        cv2.imshow("white mask", white_mask)
        return white_mask

    def find_bullseye(self, frame):
        """Finds and returns the center of the bullseye target with improved accuracy."""
        hsv = self._hsv_filter(frame)
        
        contours, hierarchy = cv2.findContours(hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print("DEBUG: No contours found")
            return None

        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        best_center = None
        max_radius = 0
        if frame.shape:
            img_height, img_width = frame.shape[:2]
        
        for i, cnt in enumerate(contours[:5]):  # Check largest 5 contours
            perimeter = cv2.arcLength(cnt, True)
            area = cv2.contourArea(cnt)
            if perimeter == 0:
                continue
            # Circularity check
            circularity = 4 * np.pi * area / (perimeter ** 2)
            if circularity < 0.5:  # More strict circularity
                continue

            # Compute centroid using image moments
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])
                radius = int(np.sqrt(area / np.pi))  # Approximate radius
                print(f"DEBUG: Centroid at ({x}, {y}) with estimated radius {radius}")
                if 15 < radius < 800:
                    max_radius = max(max_radius, radius)
                    best_center = (x, y)   
                    pixel_point = np.array([[x, y]])
                    world_coordinates = self.projection.project(
                        pixel_points=pixel_point, 
                        img_width=img_width, 
                        img_height=img_height
                    )
                    print(f"Safe Zone Center (Real-World Coordinates): {world_coordinates[0]}")

        if best_center is None:
            print("DEBUG: No valid bullseye found")
        
        return best_center