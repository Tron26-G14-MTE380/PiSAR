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
        """Detects the center of the bullseye using convex hull + enclosing circle."""
        mask = self._hsv_filter(frame)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print("DEBUG: No contours found")
            return None

        # Take the largest contour (likely the white ring)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        print("AREA = ", area)
        if area < 2000:
            print("DEBUG: Contour too small")
            return None

        # Create convex hull to complete partial rings
        hull = cv2.convexHull(largest_contour)

        # Fit minimum enclosing circle to convex shape
        (x, y), radius = cv2.minEnclosingCircle(hull)
        center = (int(x), int(y))
        radius = int(radius)

        # Optionally filter by radius range
        if radius < 20 or radius > 200:
            print("DEBUG: Radius out of expected bounds")
            return None

        # Draw debug visuals
        debug = frame.copy()
        cv2.circle(debug, center, radius, (0, 255, 0), 2)
        cv2.circle(debug, center, 3, (0, 255, 0), -1)
        cv2.putText(debug, "Bullseye", (center[0] - 40, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("Bullseye Detection (Cropped)", debug)

        # Project to world coordinates
        pixel_point = np.array([[center[0], center[1]]])
        world_coordinates = self.projection.project(
            pixel_points=pixel_point,
            img_width=frame.shape[1],
            img_height=frame.shape[0]
        )
        print(f"Bullseye Center (World): {world_coordinates[0]}")

        return center