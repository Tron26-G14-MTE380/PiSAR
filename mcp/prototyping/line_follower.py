from copy import deepcopy
from dataclasses import dataclass, field, fields, is_dataclass
import math
from typing import Optional


import cv2
import numpy as np
from debug_display import create_debug_field
from projection import RobotHomographyProjection
from scipy.spatial import cKDTree
from skimage.measure import approximate_polygon
from video_source import VideoSource

Frame = np.ndarray

@dataclass
class LineExtractorDebugData:
    line_map: Optional[Frame] = \
        create_debug_field(default=None, render=True, description="Line Map")
    
    point_map: Optional[Frame] = \
        create_debug_field(default=None, render=True, description="Line Point Map")
    
@dataclass
class SkeletonPathExtractorDebugData:
    skeleton: Optional[Frame] = \
        create_debug_field(default=None, render=True, description="Skeleton")
    
    filtered_skeleton: Optional[Frame] = \
        create_debug_field(default=None, render=True, description="Filtered Skeleton")
    
    skeleton_trajectory: Optional[Frame] = \
        create_debug_field(default=None, render=True, description="Skeleton Trajectory")
    
    skeleton_trajectory_within_edges: Optional[Frame] = \
        create_debug_field(default=None, render=True, description="Skeleton Trajectory within Edges")

@dataclass
class ProjectionDebugData:
    mapped_points: Optional[Frame] = \
        create_debug_field(default=None, render=True, description="Mapping Points")

@dataclass
class LineFollowerDebugData:
    frame: Optional[Frame] = \
        create_debug_field(default=None, render=True, description="Captured Frame")
    
    roi_frame: Optional[Frame] = \
        create_debug_field(default=None, render=True, description="Cropped Frame")
    
    preprocessed: Optional[Frame] = \
        create_debug_field(default=None, render=True, description="Preprocessed Frame")
    
    hsv_filtered: Optional[Frame] = \
        create_debug_field(default=None, render=True, description="HSV Filtered Frame")

    line_extractor_debug_data: Optional[LineExtractorDebugData] = \
        create_debug_field(default=None, render=True, description="Line Extractor Debug Data")
    
    line_points: Optional[np.ndarray] = \
        create_debug_field(default=None, render=False, description="Detected Line Points")
    
    projection_debug_data: Optional[ProjectionDebugData] = \
        create_debug_field(default=None, render=True, description="Projection Debug Data")
    
    projected_trajectory: Optional[np.ndarray] = \
        create_debug_field(default=None, render=False, description="Projected Trajectory")


@dataclass
class LineFollowerConfig:
    line_hue_masks: list[np.ndarray[(2,3)]] 
    edge_thresholds: tuple[int, int] 

def fit_trajectory(points: np.ndarray, radius=5, simplify_tol=5) -> list[np.ndarray]:
    """
    Uses RANSAC to fit multiple straight line segments, with more segments in curved areas.
    
    :param points: Nx2 NumPy array of (x, y) world coordinates.
    :param max_error: Max residual error before splitting.
    :param min_segment_length: Minimum number of points per segment.
    :param max_segments: Limit to avoid over-segmentation.
    :return: List of fitted segments [(x_coords, y_coords), ...]
    """
    
    # Build KDTree for fast nearest neighbor search
    tree = cKDTree(points)

    # Track visited points using NumPy boolean array (fast lookup)
    visited = np.zeros(len(points), dtype=bool)
    
    # Precompute nearest neighbors for all points (avoiding query repetition)
    neighbors = tree.query_ball_point(points, radius)
    
    # Convert to NumPy array for fast indexing
    max_neighbors = max(len(n) for n in neighbors)
    neighbors_array = np.full((len(points), max_neighbors), -1, dtype=int)
    
    for i, n in enumerate(neighbors):
        n_sorted = sorted(n, key=lambda j: np.linalg.norm(points[j] - points[i]))
        neighbors_array[i, :len(n_sorted)] = n_sorted

    longest_path = []
    
    def dfs(node, path):
        """Optimized Depth-First Search (DFS) for longest path extraction."""
        if visited[node]:
            return
        visited[node] = True
        path.append(points[node])

        # Use precomputed and sorted neighbors (avoids sorting in every DFS call)
        for neighbor in neighbors_array[node]:
            if neighbor != -1 and not visited[neighbor]:
                dfs(neighbor, path)

        # Store the longest path found
        nonlocal longest_path
        if len(path) > len(longest_path):
            longest_path = np.array(path)

        path.pop()
    
    for i in range(len(points)):
        if not visited[i]:
            dfs(i, [])

    if longest_path.size == 0:
        return np.empty((0, 2))  # No valid path found


    longest_path = np.array(longest_path)

    # Simplify using Ramer-Douglas-Peucker
    # simplified_path = rdp.rdp(longest_path, 3)

    simplified_path = approximate_polygon(longest_path, tolerance=simplify_tol)
    return np.array(simplified_path)


def order_points(points, radius=5):
    # Build KDTree for fast nearest neighbor search
    tree = cKDTree(points)

    # Track visited points using NumPy boolean array (fast lookup)
    visited = np.zeros(len(points), dtype=bool)
    
    # Precompute nearest neighbors for all points (avoiding query repetition)
    neighbors = tree.query_ball_point(points, radius)
    
    # Convert to NumPy array for fast indexing
    max_neighbors = max(len(n) for n in neighbors)
    neighbors_array = np.full((len(points), max_neighbors), -1, dtype=int)
    
    for i, n in enumerate(neighbors):
        n_sorted = sorted(n, key=lambda j: np.linalg.norm(points[j] - points[i]))
        neighbors_array[i, :len(n_sorted)] = n_sorted

    longest_path = []
    
    def dfs(node, path):
        """Optimized Depth-First Search (DFS) for longest path extraction."""
        if visited[node]:
            return
        visited[node] = True
        path.append(points[node])

        # Use precomputed and sorted neighbors (avoids sorting in every DFS call)
        for neighbor in neighbors_array[node]:
            if neighbor != -1 and not visited[neighbor]:
                dfs(neighbor, path)

        # Store the longest path found
        nonlocal longest_path
        if len(path) > len(longest_path):
            longest_path = np.array(path)

        path.pop()
    
    for i in range(len(points)):
        if not visited[i]:
            dfs(i, [])

    if longest_path.size == 0:
        return np.empty((0, 2))  # No valid path found

    return np.array(longest_path)

class LineFollower:
    def __init__(
            self,
            video_source: VideoSource, 
            projection: RobotHomographyProjection,
            config: LineFollowerConfig
        ):
        self.video_source = video_source
        self.projection = projection
        self.set_config(config)

    def set_config(self, config: LineFollowerConfig):
        self.config = deepcopy(config)

    def _capture_frame(self):
        return self.video_source.get_frame()

    def _extract_roi(self, frame: Frame) -> Frame:
        return frame
        height, width = frame.shape[:2]
        return frame[int(height * 0):, :]

    def _preprocess_image(self, frame: Frame) -> Frame:
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        return frame

    def _hsv_filtering(self, frame: Frame) -> Frame:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = None
        for hue_mask in self.config.line_hue_masks:
            current_mask = cv2.inRange(frame, hue_mask[0], hue_mask[1])
            if mask is not None:
                mask = cv2.bitwise_or(current_mask, mask)
            else:
                mask = current_mask

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))
        return mask

    def _edge_detection(self, frame: Frame) -> Frame:
        edges = cv2.Canny(frame, self.config.edge_thresholds[0], self.config.edge_thresholds[1], edges=0)
        kernel = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)  # Closes small gaps
    
    def _extract_path_contour(self, frame: Frame, debug_data: Optional[LineExtractorDebugData]) -> np.ndarray:
        frame = self._edge_detection(frame)

        contours, _ = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Approximate the contour to smooth sharp edges
        epsilon = 0.05 * cv2.arcLength(largest_contour, True)
        largest_contour = cv2.approxPolyDP(largest_contour, epsilon, True)

        key_points = [tuple(pt[0]) for pt in largest_contour]
    
        if debug_data is not None:
            debug_data.line_map = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            debug_data.point_map = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            # Draw the largest contour
            cv2.drawContours(debug_data.line_map, [largest_contour], -1, (0, 255, 0), 2)  # Green for selected contour

            # Mark key points on debug frame
            for point in key_points:
                cv2.circle(debug_data.point_map, point, 5, (0, 0, 255), -1)  # Red dots for key points
        
        return key_points
    
    def _skeletonize(self, frame: Frame, debug_data: Optional[SkeletonPathExtractorDebugData]) -> Frame:
        # Ensure binary format (0 and 255)
        frame = cv2.threshold(frame, 127, 255, cv2.THRESH_BINARY)[1]

        # Add 1-pixel black border
        padded_frame = np.pad(frame, ((1, 1), (1, 1)), mode="edge")

        # Thin it
        thin = cv2.ximgproc.thinning(padded_frame, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)

        # Remove the padding
        skeleton = thin[1:-1, 1:-1]

        if debug_data:
            debug_data.skeleton = skeleton

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(skeleton, connectivity=8)

        if num_labels <= 1:
            return skeleton  # No valid components

        # Find the largest non-background component
        largest_idx = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])  # Ignore background (0)
        largest_skeleton = np.zeros_like(skeleton)
        largest_skeleton[labels == largest_idx] = 255

        if debug_data:
            debug_data.filtered_skeleton = largest_skeleton
 
        return largest_skeleton

    def _fit_skeleton(self, skeleton: Frame, debug_data: Optional[SkeletonPathExtractorDebugData]):

        # Extract centerline points from skeleton
        points = np.where(skeleton > 0)
        points = np.column_stack((points[1], points[0]))

        segments_points = order_points(points)

        # Simplify trajectory into polygon
        segments_points = approximate_polygon(segments_points, tolerance=5)

        if debug_data:

            # Convert grayscale skeleton to BGR for visualization
            debug_data.skeleton_trajectory = np.zeros(skeleton.shape, dtype=np.uint8)

            # Draw on debug image
            for i in range(len(segments_points) - 1):
                p1 = (int(segments_points[i, 0]), int(segments_points[i, 1]))
                p2 = (int(segments_points[i+1, 0]), int(segments_points[i+1, 1]))
                cv2.line(debug_data.skeleton_trajectory, p1, p2, 255, 1)
            
            for i in range(len(segments_points) - 1):
                p = (int(segments_points[i, 0]), int(segments_points[i, 1]))
                cv2.circle(debug_data.skeleton_trajectory, p, 2, 255, 2)

        return segments_points
    
    def _extract_path_skeleton(self, frame: Frame, debug_data: Optional[SkeletonPathExtractorDebugData]) -> np.ndarray:
        trajectory = self._fit_skeleton(self._skeletonize(frame, debug_data), debug_data)


        if debug_data:
            edge_map = self._edge_detection(frame)
            debug_data.skeleton_trajectory_within_edges = np.maximum(edge_map, debug_data.skeleton_trajectory)

        return trajectory

    def _project_to_world(self, pixel_points: np.ndarray, width: int, height: int, debug_data: Optional[ProjectionDebugData]) -> np.ndarray:
        world_points = self.projection.project(pixel_points, width, height)

        if debug_data:
            debug_data.mapped_points = np.zeros(shape=(height, width, 3), dtype=np.uint8)

            world_ref = np.array([[0], [0], [0]]).T
            camera_position = self.projection.camera_position.T

            # Step 1: Determine world bounds by projecting edge pixels along the image border
            edge_pixels = np.array([[x, y] for x in range(0, width, 10) for y in [0, height]] +  # Top & Bottom edges
                                    [[x, y] for x in [0, width] for y in range(0, height, 10)])  # Left & Right edges

            u_grid, v_grid = np.meshgrid(np.linspace(0, width, 10), np.linspace(0, height, 10))
            camera_view_grid_image_points = np.vstack([u_grid.ravel(), v_grid.ravel()]).T
            camera_view_grid_world_points = self.projection.project(camera_view_grid_image_points, width, height)

            all_world_points = np.vstack([world_ref[0, :2], camera_view_grid_world_points])
            all_world_points_x = all_world_points[:, 0]
            all_world_points_y = all_world_points[:, 1]

            # Get accurate world bounds based on projected edges
            X_min, X_max = np.min(all_world_points_x), np.max(all_world_points_x)
            Y_min, Y_max = np.min(all_world_points_y), np.max(all_world_points_y)

            # Compute scaling factors
            world_x_range = X_max - X_min
            world_y_range = Y_max - Y_min

            # Scale factors based on image size
            world_to_image_scale_factor_x = width / world_x_range
            world_to_image_scale_factor_y = height / world_y_range

            # Use the smaller scale factor to ensure everything fits while keeping reference fixed
            world_to_image_scale_factor = min(world_to_image_scale_factor_x, world_to_image_scale_factor_y)

            # Step 2: Scale world points while keeping `camera_position` at (w/2, h)
            def world_to_image_fixed(points):
                """ Map world coordinates to the image while keeping camera_position at bottom-center """

                # Convert to image coordinates
                x_scaled = points[:, 0] * world_to_image_scale_factor + width / 2
                y_scaled = height - (points[:, 1] * world_to_image_scale_factor)  # Invert Y for image

                return np.vstack((x_scaled, y_scaled)).T
            
            camera_view_grid_image_points = world_to_image_fixed(camera_view_grid_world_points)
            image_points = world_to_image_fixed(world_points) 
            image_camera_point = world_to_image_fixed(camera_position)
            image_ref_point = world_to_image_fixed(world_ref) 

          # Draw grid points
            for (px, py) in camera_view_grid_image_points:
                cv2.circle(debug_data.mapped_points, (int(px), int(py)), radius=1, color=(255, 255, 255), thickness=-1)  # white points

            # Draw projected points
            for (px, py) in image_points:
                cv2.circle(debug_data.mapped_points, (int(px), int(py)), radius=5, color=(0, 0, 255), thickness=-1)  # Red points
            
            # Draw bottom-center reference point
            cv2.circle(debug_data.mapped_points, (int(image_ref_point[0, 0]), int(image_ref_point[0, 1])), radius=6, color=(255, 0, 0), thickness=-1)  # Blue refer

            # Draw bottom-center reference point
            cv2.circle(debug_data.mapped_points, (int(image_camera_point[0, 0]), int(image_camera_point[0, 1])), radius=6, color=(0, 255, 0), thickness=-1)  # Blue refer

        return world_points

    def generate_trajectory(self, debug_data: LineFollowerDebugData):
        frame = self._capture_frame()
        if frame is None:
            return None
        
        img_height, img_width = frame.shape[:2]
        if debug_data:
            debug_data.frame = frame.copy()

        frame = self._extract_roi(frame)
        if debug_data:
            debug_data.roi_frame = frame.copy()
        
        frame = self._preprocess_image(frame)
        if debug_data:
            debug_data.preprocessed = frame.copy()
            
        frame = self._hsv_filtering(frame)
        if debug_data:
            debug_data.hsv_filtered = frame.copy()

        if debug_data:
            debug_data.line_extractor_debug_data = SkeletonPathExtractorDebugData()
        image_points = self._extract_path_skeleton(frame, debug_data=(debug_data.line_extractor_debug_data if debug_data else None))
        
        if debug_data:
            debug_data.projection_debug_data = ProjectionDebugData()
        trajectory = self._project_to_world(image_points, img_width, img_height, (debug_data.projection_debug_data if debug_data else None))

        if debug_data:
            debug_data.projected_trajectory = trajectory.copy()

        return trajectory

