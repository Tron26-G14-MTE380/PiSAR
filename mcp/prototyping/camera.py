from dataclasses import dataclass
from functools import cache

import numpy as np


@dataclass(frozen=True)
class CameraCalibration:
    calibration_img_width: int
    calibration_img_height: int
    focal_x: float
    focal_y: float
    center_x: float
    center_y: float
    skew: float

    @cache
    def get_transformation(self, width: int, height: int) -> np.ndarray[(3, 3)]:
        scale_x = width / self.calibration_img_width
        scale_y = height / self.calibration_img_height

        return np.array([
            [scale_x, 0, 0],
            [0, scale_y, 0],
            [0, 0, 1]
        ]) @ np.array([
            [self.focal_x, self.skew, self.center_x],
            [0, self.focal_y, self.center_y],
            [0, 0, 1]
        ])

    @cache
    def get_inverse_transformation(self, width: int, height: int) -> np.ndarray[(3, 3)]:
        return np.linalg.inv(self.get_transformation(width, height))