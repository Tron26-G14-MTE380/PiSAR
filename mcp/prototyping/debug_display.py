
from dataclasses import field, fields, is_dataclass
import math
import cv2
import numpy as np


def is_array_like(obj) -> bool:
    """Checks if an object is array-like (can be indexed and has a length)."""
    try:
        # Must have a length and support indexing
        len(obj)
        obj[0]
        return True
    except (TypeError, IndexError, KeyError):
        return False

def get_debug_images(debug_data):
    debug_images = []
    for field in fields(debug_data):

        if field.metadata and "render" in field.metadata and not bool(field.metadata["render"]):
            continue

        field_name = field.name
        field_value = getattr(debug_data, field_name)
        if is_dataclass(field_value):
            field_images = get_debug_images(field_value)
            if field_images:
                debug_images.extend(field_images)
        elif is_array_like(field_value) and field_value.ndim in [2, 3]:
            debug_image = field_value

            debug_image_descr = field_name

            if field.metadata and "description" in field.metadata:
                debug_image_descr = field.metadata["description"]
            
            debug_images.append((debug_image_descr, debug_image))
    
    return debug_images

def create_debug_frame(debug_data, grid_padding: int = 50):
    """create debug images in a grid format using OpenCV."""

    
    debug_images = get_debug_images(debug_data)

    labels = [i[0] for i in debug_images]
    images = [i[1] for i in debug_images]

    if not debug_images:
        return None

    # Determine the smallest scaling factor to apply to all images
    min_height = min(img.shape[0] for img in images)
    min_width = min(img.shape[1] for img in images)
    scale_factor = 320 / max(min_width, min_height)  # Ensures no image exceeds 640px in width/height

    # Resize images while maintaining aspect ratio
    resized_images = [cv2.resize(image, (int(image.shape[1] * scale_factor), int(image.shape[0] * scale_factor)))
                    for image in images]

    # Convert grayscale to BGR for consistency in display
    for i, image in enumerate(resized_images):
        if len(image.shape) == 2:  # Grayscale image
            resized_images[i] = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    # Determine grid layout
    num_images = len(resized_images)
    grid_cols = 4  # Number of columns in the grid
    grid_rows = math.ceil((num_images + 1) / grid_cols)  # Compute required rows

    img_height, img_width = next(iter(resized_images)).shape[:2]

    # Create an empty canvas for the grid with padding
    grid_height = (img_height + grid_padding) * grid_rows - grid_padding
    grid_width = (img_width + grid_padding) * grid_cols - grid_padding
    grid_canvas = np.full((grid_height, grid_width, 3), 255, dtype=np.uint8)

    # Place images in the grid with padding
    for idx, (key, img) in enumerate(zip(labels, resized_images)):
        row, col = divmod(idx, grid_cols)
        y_start = row * (img_height + grid_padding)
        x_start = col * (img_width + grid_padding)
        grid_canvas[y_start:y_start+img.shape[0], x_start:x_start+img.shape[1]] = img

        # Add text labels
        cv2.putText(grid_canvas, key, (x_start + 10, y_start + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    
    return grid_canvas


def create_debug_field(default = None, render: bool = True, description=None):
    metadata = {"render": render}
    if description:
        metadata["description"] = description
    return field(default=default, metadata=metadata)
