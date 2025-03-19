import cv2
import numpy as np
from projection import RobotHomographyProjection

class SafeZoneDetector:
    """Detects and reconstructs a green rectangular safe zone in an image, even if partially occluded."""

    def __init__(self, projection: RobotHomographyProjection):
        """Initializes the safe zone detector with a projection model."""
        self.projection = projection

    def is_safe_zone_in_scene(self, frame) -> bool:
        """Checks if a safe zone is present by verifying if `find_safe_zone()` finds a valid rectangle."""
        center = self.find_safe_zone(frame)
        return center is not None  # Return True only if a valid rectangular safe zone is found

    def find_safe_zone(self, frame):
        """Finds the safe zone, ensuring it's roughly rectangular before computing the centroid."""
        if frame is None or not hasattr(frame, "shape"):
            print("Error: Invalid frame passed to find_safe_zone!")
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, (40, 50, 50), (80, 255, 255))

        # Show initial mask
        cv2.imshow("Green Mask", green_mask)
        cv2.waitKey(1)

        # Find contours
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"Contours found: {len(contours)}")

        if not contours:
            print("No safe zone detected.")
            return None

        # Merge contours using convex hull
        all_points = np.vstack([c.reshape(-1, 2) for c in contours])
        hull = cv2.convexHull(all_points).reshape(-1, 2)

        # Approximate the shape to simplify it
        epsilon = 0.02 * cv2.arcLength(hull, True)
        approx = cv2.approxPolyDP(hull, epsilon, True)

        # Debug: Draw the convex hull on the frame
        debug_image = frame.copy()
        cv2.drawContours(debug_image, [hull], -1, (255, 255, 255), 2)
        cv2.imshow("Convex Hull Shape", debug_image)
        cv2.waitKey(1)

        # Validate that the shape is a rectangle
        if len(approx) < 4 or len(approx) > 8:
            print("Rejected: Shape does not have 4-8 vertices (not a rectangle)")
            return None

        # Compute bounding box
        x, y, w, h = cv2.boundingRect(hull)
        hull_area = cv2.contourArea(hull)
        bounding_box_area = w * h

        print(f"Hull area: {hull_area:.2f}, Bounding box area: {bounding_box_area:.2f}")

        if hull_area < 300 or bounding_box_area < 1000:
            print("Rejected: Safe zone area too small")
            return None

        # Compute centroid
        cx, cy = x + w // 2, y + h // 2
        print(f"Safe Zone Center (Pixel Coordinates): ({cx}, {cy})")

        pixel_point = np.array([[cx, cy]])

        if frame.shape:
            img_height, img_width = frame.shape[:2]
        else:
            print("Error: Frame has no valid shape!")
            return None

        # Debugging projection instance
        print(f"Projection instance type: {type(self.projection)}")
        print(f"Projecting: Pixel {pixel_point}, Width: {img_width}, Height: {img_height}")

        try:
            world_coordinates = self.projection.project(
                
                pixel_points=pixel_point, 
                img_width=img_width, 
                img_height=img_height
            )
            print(f"Safe Zone Center (Real-World Coordinates): {world_coordinates[0]}")
        except Exception as e:
            print(f"Projection error: {e}")
            world_coordinates = None

        # Return pixel coordinates instead
        return (cx, cy)  # Return pixel coordinates