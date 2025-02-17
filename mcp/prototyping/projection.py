import math
import numpy as np

from camera import CameraCalibration


class RobotHomographyProjection:
    def __init__(self, camera_calibration: CameraCalibration, camera_position: np.ndarray[(3, 1)], camera_tilt_angle: float, axis_mapping: np.ndarray[(3, 3)]):
        self._translation = camera_position
        self._rotation = axis_mapping @ np.array([
            [1, 0, 0],
            [0, math.cos(camera_tilt_angle), math.sin(camera_tilt_angle)],
            [0, -math.sin(camera_tilt_angle), math.cos(camera_tilt_angle)]
        ])

        self._calibration = camera_calibration
        self._inverse_rotation = np.linalg.inv(self._rotation)

    @property
    def camera_position(self):
        return self._translation.copy()

    def project(self, pixel_points: np.ndarray, img_width: float, img_height: float):
        world_points = np.zeros(shape=pixel_points.shape)

        # Inverted intrinsic camera transformation
        inverse_k = self._calibration.get_inverse_transformation(img_width, img_height)
        total_inverse_projection = self._inverse_rotation @ inverse_k

        for i, (u, v) in enumerate(pixel_points):
            pixel_coords = np.array([[u], [v], [1]])  # Homogeneous coordinates
            
            p = total_inverse_projection @ pixel_coords
            zc = self._translation[2, 0] / p[2, 0]
            world_points[i, :] = np.transpose(p[:2, 0] * zc + self._translation[:2, 0])

        return world_points



