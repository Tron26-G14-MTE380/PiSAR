import sys
import time
import cv2
import numpy as np
from debug_display import create_debug_frame
from line_follower import LineFollower, LineFollowerConfig, LineFollowerDebugData
from camera import CameraCalibration
from projection import RobotHomographyProjection
from video_source import RepeatedImageFileSource, VideoCameraSource

debug = False

resolution_width = 320
resolution_height = 240

camera_height=45
camera_offset=5
camera_tilt_angle=np.deg2rad(40)
camera_axis_mapping=np.array([
    [1, 0, 0],
    [0, 0, 1],
    [0, 1, 0]
])

camera_calibration = CameraCalibration(
    calibration_img_width=640,
    calibration_img_height=480,
    focal_x=450,
    focal_y=450,
    center_x=320,
    center_y=240,
    skew=0
)

projection = RobotHomographyProjection(
    camera_calibration=camera_calibration, 
    camera_position=np.array([[0], [camera_offset], [camera_height]]),
    camera_tilt_angle=camera_tilt_angle,
    axis_mapping=camera_axis_mapping
)

tracker_config = LineFollowerConfig(
    line_hue_masks=[
        np.array([[0, 120, 70], [10, 255, 255]]),
        np.array([[170, 120, 70], [180, 255, 255]]),
    ],
    edge_thresholds=[50, 150]
)

def display_debug_data(debug_data: LineFollowerDebugData, padding=10):
    
    grid_canvas = create_debug_frame(debug_data, grid_padding=padding)

    # Show the composite debug display
    cv2.imshow("Debug Data", grid_canvas)
    key = cv2.waitKey(1)  # Small delay to allow window to refresh

    if key == 27 or cv2.getWindowProperty("Debug Data", cv2.WND_PROP_VISIBLE) < 1:  # ESC key or window closed
        print("Debug window closed. Exiting program.")
        cv2.destroyAllWindows()
        sys.exit()


if __name__ == "__main__":

    resolution = (resolution_width, resolution_height)
    #video_source = VideoCameraSource(resolution=resolution)
    video_source = RepeatedImageFileSource("sample_images/red_tape2.jpg", resolution=resolution)
    with video_source:
        
        tracker = LineFollower(video_source, projection=projection, config=tracker_config)
        
        debug_data = LineFollowerDebugData() if debug else None

        start = time.time()
        time.sleep(0.1)

        while True:  # Run for 10 frames
            end = time.time()
            fps = 1 / (end - start)
            print(f"Fps: {fps:.5}")

            start = time.time()
            trajectory = tracker.generate_trajectory(debug_data)

            if trajectory is None:
                raise Exception("End of video")
            
            if debug:
                display_debug_data(debug_data)
