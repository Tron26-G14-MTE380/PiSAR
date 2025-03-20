import cv2
import numpy as np
import os
from projection import RobotHomographyProjection
from camera import CameraCalibration

class DotDetector:
    """Detects black dots in an image and projects them to world coordinates."""
    
    def __init__(self, projection: RobotHomographyProjection):
        self.projection = projection
    
    def process_image(self, frame: np.ndarray):
        """Processes the image to detect black dots and return their pixel coordinates."""
        if frame is None:
            print("Error: Image not loaded.")
            return []
        
        # Convert image to grayscale for better contrast
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply adaptive thresholding to extract black dots
        _, mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        
        # Apply morphological operations to remove noise
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours of the detected dots
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        pixel_points = []
        if frame.shape:
            img_height, img_width = frame.shape[:2]
        
        for contour in contours:
            # Get the center of each dot
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                pixel_points.append((cx, cy))
                
                # Draw detected points and overlay coordinates
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"({cx}, {cy})", (cx+10, cy-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Display the processed image
        cv2.imshow("Filtered Image", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        world_coordinates = self.projection.project(
            pixel_points=np.array(pixel_points), 
            img_width=img_width, 
            img_height=img_height
        )
        
        return pixel_points, world_coordinates

if __name__ == "__main__":
    from projection import RobotHomographyProjection
    
    # Initialize projection system (parameters must be configured)
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

    projection_instance = RobotHomographyProjection(
        camera_calibration=camera_calibration, 
        camera_position=np.array([[0], [camera_offset], [camera_height]]),
        camera_tilt_angle=camera_tilt_angle,
        axis_mapping=camera_axis_mapping
    )
    detector = DotDetector(projection_instance)
    
    image_path = os.path.join("..", "sample_images", "homography_validation.jpg")
    frame = cv2.imread(image_path)
    
    pixel_coords, world_coords = detector.process_image(frame)
    
    print("Pixel Coordinates of detected dots:", pixel_coords)
    print("World Coordinates of detected dots:", world_coords)
