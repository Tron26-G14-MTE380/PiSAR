import time
import cv2
import numpy as np
from abc import ABC, abstractmethod

try:
    from picamera2 import Picamera2
    LIBCAMERA_AVAILABLE = True
except ImportError:
    LIBCAMERA_AVAILABLE = False


class VideoSource(ABC):
    def __init__(self, resolution: tuple[int, int] = (640, 480)):
        self.resolution = resolution

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def get_frame(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    def __enter__(self):
        return self.start()

    def __exit__(self, *args):
        self.stop()

class RepeatedImageFileSource(VideoSource):
        def __init__(self, file_path: str, resolution=(640, 480)):
            super().__init__(resolution)
            self.file_path = file_path
            self.img = None

        def start(self):
            if self.img:
                raise Exception("Already started")
            
            self.img = cv2.imread(self.file_path)
            self.img = cv2.resize(self.img, self.resolution)

        def get_frame(self):
            return self.img.copy()

        def stop(self):
            if self.img is not None:
                self.img = None

if LIBCAMERA_AVAILABLE:

    class VideoCameraSource(VideoSource):
        def __init__(self, resolution=(640, 480)):
            super().__init__(resolution)
            self.camera = None

        def start(self):
            if self.camera:
                raise Exception("Already started")
            
            self.camera = Picamera2()
            video_config = self.camera.create_video_configuration(main={"size": self.resolution})
            self.camera.start(config=video_config)
            time.sleep(1)

        def get_frame(self):
            ret = False
            while not ret:
                ret, frame = self.camera.capture_array("main")
            return frame

        def stop(self):
            if self.camera:
                self.camera.stop()
                self.camera = None

else:

    class VideoCameraSource(VideoSource):
        def __init__(self, resolution=(640, 480)):
            super().__init__(resolution)
            self.cap = None

        def start(self):
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

        def get_frame(self):
            ret = False
            while not ret:
                ret, frame = self.cap.read()
            
            return frame

        def stop(self):
            if self.cap:
                self.cap.release()
                self.cap = None