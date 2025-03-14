import numpy as np

def pixel_to_angle(calibration_file, pixel):
    """
    Converts a pixel location to angles (theta_x, theta_y) with respect to the camera's optical axis.
    Assumes the image has already been undistorted.

    Args:
        calibration_file (str): Path to the .npz file containing calibration data.
        pixel (tuple): (x, y) pixel coordinates in the undistorted image.

    Returns:
        tuple: (theta_x, theta_y) angles in radians.
    """

    # Load camera calibration data
    with np.load(calibration_file) as data:
        camera_matrix = data["camera_matrix"]

    # Extract focal lengths and optical center from the camera matrix
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]  # Focal lengths
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]  # Optical center

    # Convert pixel coordinates to normalized camera coordinates
    x_norm = (pixel[0] - cx) / fx
    y_norm = (pixel[1] - cy) / fy

    # Compute angles using inverse tangent
    theta_x = np.arctan(x_norm)  # Horizontal angle
    theta_y = np.arctan(y_norm)  # Vertical angle

    return theta_x, theta_y
