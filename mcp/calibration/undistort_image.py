import cv2
import numpy as np

# Camera intrinsic matrix (from calibration)
K = np.array([
    [2630.73, 0, 1589.29],  # fx, skew, cx
    [0, 2621.95, 1085.99],  # 0, fy, cy
    [0, 0, 1]               # 0, 0, 1
])

# Distortion coefficients (from calibration)
dist_coeffs = np.array([0.13149815, 0.40868211, -0.00805099, -0.00454671, -2.73795495])

def undistort_image(input_path: str, output_path: str):
    """
    Undistorts an image using camera calibration parameters.

    :param input_path: Path to the distorted image.
    :param output_path: Path to save the undistorted image.
    """
    # Load the image
    image = cv2.imread(input_path)
    if image is None:
        print(f"Error: Unable to load image '{input_path}'")
        return

    # Get optimal new camera matrix
    h, w = image.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist_coeffs, (w, h), 1, (w, h))

    # Apply undistortion
    undistorted = cv2.undistort(image, K, dist_coeffs, None, new_K)

    # Crop the image based on ROI (optional)
    x, y, w, h = roi
    undistorted = undistorted[y:y+h, x:x+w]

    # Save the result
    cv2.imwrite(output_path, undistorted)
    print(f"✅ Undistorted image saved to: {output_path}")

# Example usage
undistort_image("captured_images/homography_validation.jpg", "captured_images/homography_validation_undistorted.jpg")
