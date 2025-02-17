

import time
import numpy as np
import cv2
from threading import Event, Lock, Thread

def sum_neighbors(matrix: np.ndarray, row: int, col: int) -> int:
    """
    Computes the sum of the 8 neighboring elements of a given pixel in the matrix.
    Assumes the pixel is not on the edge or corner.

    :param matrix: 2D NumPy array
    :param row: Row index of the target element
    :param col: Column index of the target element
    :return: Sum of the 8 neighboring elements
    """
    p1 = int(matrix[row-1, col])
    p2 = int(matrix[row-1, col+1])
    p3 = int(matrix[row, col+1])
    p4 = int(matrix[row+1, col+1])
    p5 = int(matrix[row+1, col])
    p6 = int(matrix[row+1, col-1])
    p7 = int(matrix[row, col-1])
    p8 = int(matrix[row-1, col-1])
    return int(p1 + p2 + p3 + p4 + p5 + p6 + p7 + p8)

def num_ordered_pairs(matrix: np.ndarray, row: int, col: int) -> int:
    p1 = int(matrix[row-1, col])
    p2 = int(matrix[row-1, col+1])
    p3 = int(matrix[row, col+1])
    p4 = int(matrix[row+1, col+1])
    p5 = int(matrix[row+1, col])
    p6 = int(matrix[row+1, col-1])
    p7 = int(matrix[row, col-1])
    p8 = int(matrix[row-1, col-1])
    
    return (
        (p1 == 0 and p2 == 1) + (p2 == 0 and p3 == 1) +
        (p3 == 0 and p4 == 1) + (p4 == 0 and p5 == 1) +
        (p5 == 0 and p6 == 1) + (p6 == 0 and p7 == 1) +
        (p7 == 0 and p8 == 1) + (p8 == 0 and p1 == 1)
    )

def zhang_condition(img: np.ndarray, r: int, c: int):
    p1 = int(img[r-1, c])
    p2 = int(img[r-1, c+1])
    p3 = int(img[r, c+1])
    p4 = int(img[r+1, c+1])
    p5 = int(img[r+1, c])
    p6 = int(img[r+1, c-1])
    p7 = int(img[r, c-1])
    p8 = int(img[r-1, c-1])

    return (
        (p1 == 1 and p3 == 1 and p5 == 1) == 0 and
        (p3 == 1 and p5 == 1 and p7 == 1) == 0
    )


def check_template_a(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r, c-1]) == 0 and
        int(img[r, c+1]) == 1 and
        int(img[r, c+2]) == 0 and
        int(img[r+1, c]) == 1
    )

def check_template_b(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r-1, c]) == 0 and
        int(img[r, c+1]) == 1 and
        int(img[r+1, c]) == 1 and
        int(img[r+2, c]) == 0
    )

def check_template_c(img: np.ndarray, r: int, c: int):
    
    matches = bool(
        int(img[r-1, c]) == 0 and
        int(img[r, c-1]) == 0 and
        int(img[r, c+1]) == 1 and
        int(img[r+1, c]) == 1
    )

    if not matches:
        return False

    num_matching_ts = (
        int(int(img[r-1, c+2]) == 1) +
        int(int(img[r, c+2]) == 1) +
        int(int(img[r+1, c+2]) == 1) +
        int(int(img[r+2, c-1]) == 1) +
        int(int(img[r+2, c]) == 1) +
        int(int(img[r+2, c+1]) == 1)
    )

    return num_matching_ts >= 2

    

def check_template_d(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r-2, c-1]) == 0 and
        int(img[r-2, c]) == 0 and
        int(img[r-2, c+1]) == 0 and

        int(img[r-1, c-1]) == 0 and
        int(img[r-1, c]) == 1 and
        int(img[r-1, c+1]) == 0 and

        int(img[r, c-1]) == 0 and
        int(img[r, c+1]) == 1 and

        int(img[r+1, c]) == 1 and
        int(img[r+1, c+1]) == 1 and

        int(img[r, c+2]) == int(img[r+1, c+2]) and
        int(img[r+2, c]) == int(img[r+2, c+1]) and
        int(sum(int(img[r, c+2]), int(img[r+1, c+2]), int(img[r+2, c]), int(img[r+2, c+1]))) > 1
    )

def check_template_e(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r-1, c-2]) == 0 and
        int(img[r-1, c-1]) == 0 and
        int(img[r-1, c]) == 0 and

        int(img[r, c-2]) == 0 and
        int(img[r, c-1]) == 1 and
        int(img[r, c+1]) == 1 and

        int(img[r+1, c-2]) == 0 and
        int(img[r+1, c-1]) == 0 and
        int(img[r+1, c]) == 1 and
        int(img[r+1, c+1]) == 1 and

        int(img[r, c+2]) == int(img[r+1, c+2]) and
        int(img[r+2, c]) == int(img[r+2, c+1]) and
        int(sum(int(img[r, c+2]), int(img[r+1, c+2]), int(img[r+2, c]), int(img[r+2, c+1]))) > 1
    ) 

def check_template_f(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r-1, c-2]) == 0 and
        int(img[r-1, c-1]) == 0 and
        int(img[r-1, c]) == 1 and
        int(img[r-1, c+1]) == 1 and

        int(img[r, c-2]) == 0 and
        int(img[r, c-1]) == 1 and
        int(img[r, c+1]) == 1 and

        int(img[r+1, c-2]) == 0 and
        int(img[r+1, c-1]) == 0 and
        int(img[r+1, c]) == 0 and

        int(img[r-2, c]) == int(img[r-2, c+1]) and
        int(img[r, c+2]) == int(img[r-1, c+2])
    )

def check_template_g(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r-1, c-1]) == 1 and
        int(img[r-1, c]) == 1 and
        int(img[r-1, c+1]) == 0 and
        int(img[r-1, c+2]) == 0 and

        int(img[r, c-1]) == 1 and
        int(img[r, c+1]) == 1 and
        int(img[r, c+2]) == 0 and

        int(img[r+1, c]) == 0 and
        int(img[r+1, c+1]) == 0 and
        int(img[r+1, c+2]) == 0 and

        int(img[r-2, c]) == int(img[r-2, c-1]) and
        int(img[r, c-2]) == int(img[r-1, c-2])
    )

def check_template_h(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r-1, c]) == 0 and
        int(img[r-1, c+1]) == 0 and
        int(img[r-1, c+2]) == 0 and

        int(img[r, c-1]) == 1 and
        int(img[r, c+1]) == 1 and
        int(img[r, c+2]) == 0 and

        int(img[r+1, c-1]) == 1 and
        int(img[r+1, c]) == 1 and
        int(img[r+1, c+1]) == 0 and
        int(img[r+1, c+2]) == 0 and

        int(img[r, c-2]) == int(img[r+1, c-2]) and
        int(img[r+2, c]) == int(img[r+2, c-1])
    )

def check_template_i(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r-2, c-1]) == 0 and
        int(img[r-2, c]) == 0 and
        int(img[r-2, c+1]) == 0 and

        int(img[r-1, c-1]) == 0 and
        int(img[r-1, c]) == 1 and
        int(img[r-1, c+1]) == 0 and

        int(img[r, c-1]) == 1 and
        int(img[r, c+1]) == 0 and
        
        int(img[r+1, c-1]) == 1 and
        int(img[r+1, c]) == 1 and
        
        int(img[r, c-2]) == int(img[r+1, c-2]) and
        int(img[r+2, c]) == int(img[r+2, c-1])
    )

def check_template_j(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r-1, c]) == 1 and
        int(img[r-1, c+1]) == 1 and

        int(img[r, c-1]) == 0 and
        int(img[r, c+1]) == 1 and

        int(img[r+1, c-1]) == 0 and
        int(img[r+1, c]) == 1 and
        int(img[r+1, c+1]) == 0 and

        int(img[r+2, c-1]) == 0 and
        int(img[r+2, c]) == 0 and
        int(img[r+2, c+1]) == 0 and

        int(img[r-2, c]) == int(img[r-2, c+1]) and
        int(img[r, c+2]) == int(img[r-1, c+2])
    )

def check_template_k(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r-1, c-1]) == 1 and
        int(img[r-1, c]) == 1 and

        int(img[r, c-1]) == 1 and
        int(img[r, c+1]) == 0 and

        int(img[r+1, c-1]) == 0 and
        int(img[r+1, c]) == 1 and 
        int(img[r+1, c+1]) == 0 and

        int(img[r+2, c-1]) == 0 and
        int(img[r+2, c]) == 0 and 
        int(img[r+2, c+1]) == 0 and
        
        int(img[r-2, c]) == int(img[r-2, c-1]) and
        int(img[r, c-2]) == int(img[r-1, c-2])
    )

def check_template_l(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r-1, c-1]) == 1 and
        int(img[r-1, c]) == 1 and
        int(img[r-1, c+1]) == 0 and

        int(img[r, c-1]) == 0 and
        int(img[r, c+1]) == 1 and

        int(img[r+1, c-1]) == 0 and
        int(img[r+1, c]) == 0 and
        int(img[r+1, c+1]) == 1 
    )

def check_template_m(img: np.ndarray, r: int, c: int):
    return bool(
        int(img[r-1, c-1]) == 0 and
        int(img[r-1, c]) == 1 and
        int(img[r-1, c+1]) == 1 and

        int(img[r, c-1]) == 1 and
        int(img[r, c+1]) == 0 and

        int(img[r+1, c-1]) == 1 and
        int(img[r+1, c]) == 0 and
        int(img[r+1, c+1]) == 0 
    )

def match_deleting_template(img: np.ndarray, r: int, c: int):

    deleting_template_checkers = [
        check_template_d, 
        check_template_e, 
        check_template_f, 
        check_template_g, 
        check_template_h, 
        check_template_i, 
        check_template_j,
        check_template_k,
        check_template_l, 
        check_template_m 
    ]

    matches = 0
    for checker in deleting_template_checkers:
        if checker(img, r, c):
            return True
    
    return False

class SkeletonizationVisualization:

    def __init__(self, img: np.ndarray, enable: bool=True):
        self._img = cv2.cvtColor(img.copy() * 255, cv2.COLOR_GRAY2BGR)
        self._display_img = self._img
        
        self._stop_flag = Event()
        self._img_lock = Lock()
        self._thread = Thread(target=self._render_thread)
        self._enable = enable

    def start(self):
        if self._enable:
            self._stop_flag.clear()
            self._thread.start()
            self._draw(self._img)

    def stop(self):
        if self._enable:
            self._stop_flag.set()
            self._thread.join()

    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, *args):
        self.stop()

    def inspecting_at(self, r: int, c: int):
        if self._enable:
            temp = self._img.copy()
            
            cv2.circle(temp, (c, r), 3, (255, 255, 0), 3)
            self._draw(temp)

    def deleted(self, r: int, c: int):
        if self._enable:
            cv2.circle(self._img, (c, r), 1, (0, 0, 255), 1)
            self._draw(self._img)

    def update(self, img: np.ndarray):
        if self._enable:
            self._img = cv2.cvtColor(img.copy() * 255, cv2.COLOR_GRAY2BGR)
            self._draw(self._img)

    def _draw(self, img: np.ndarray):
        with self._img_lock:
            self._display_img = img

    def _render_thread(self):
        while not self._stop_flag.is_set():
            with self._img_lock:
                img = self._display_img.copy()
            cv2.imshow("debug", img)
            cv2.waitKey(1)

def jun_ma(img: np.ndarray):
    img = img.copy()
    row, col = img.shape
    delete_matrix = np.zeros(img.shape, dtype=np.uint8)
    
    with SkeletonizationVisualization(img, enable=False) as visualization:
        i = 1
        while True:
            print(f"Iteration {i}")
            flag = False

            for r in range(3, row - 2 + 1):
                for c in range(3, col - 2 + 1):
                    p = int(img[r, c])
                    visualization.inspecting_at(r, c)
                    if p != 0: # Foreground Pixel
                        A = num_ordered_pairs(img, r, c)
                        B = sum_neighbors(img, r, c)
                        if A == 1 and (B >= 2 and B <= 6):
                            restore = check_template_a(img, r, c) or check_template_b(img, r, c)
                            compulsory_delete = check_template_c(img, r, c)
                            if not restore or compulsory_delete:
                                delete_matrix[r, c] = 1
                                flag = True
                                visualization.deleted(r, c)
                        elif A == 2 and (B >= 4 and B <= 5):
                            if match_deleting_template(img, r, c):
                                delete_matrix[r, c] = 1
                                flag = True
                                visualization.deleted(r, c)
            if flag == False:
                break

            img = img - delete_matrix
            delete_matrix = np.zeros(img.shape, dtype=np.uint8)
            visualization.update(img)
            i+=1

        # Post processing
        for r in range(3, row - 2 + 1):
            for c in range(3, col - 2 + 1): 
                p = int(img[r, c])
                visualization.inspecting_at(r, c)

                p1 = int(img[r-1, c])
                p2 = int(img[r-1, c+1])
                p3 = int(img[r, c+1])
                p4 = int(img[r+1, c+1])
                p5 = int(img[r+1, c])
                p6 = int(img[r+1, c-1])
                p7 = int(img[r, c-1])
                p8 = int(img[r-1, c-1])

                if (
                    (p1 == 1 and p3 == 1 and p6 == 0) or
                    (p3 == 1 and p5 == 1 and p8 == 0) or
                    (p5 == 1 and p7 == 1 and p2 == 0) or
                    (p7 == 1 and p1 == 1 and p4 == 0)
                ):
                    img[r, c] = 0
                    visualization.deleted(r, c)
        
        visualization.update(img)
        return img


# Load the image in grayscale
image = cv2.imread("prototyping/skeletonization/letters.jpg", cv2.IMREAD_GRAYSCALE)

# Convert to boolean matrix (threshold at 128)
boolean_matrix = image > 128

# Convert to 1s and 0s (optional, if needed explicitly as int)
binary_image = boolean_matrix.astype(np.uint8)

skeleton = jun_ma(binary_image)

skeleton = (skeleton * 255).astype(np.uint8)

cv2.imshow("debug", skeleton)

cv2.waitKey(10000)