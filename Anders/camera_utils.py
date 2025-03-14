# camera_utils.py

import cv2
import numpy as np
import os


def take_image(max_attempts=5):
    """
    Attempts to capture an image from the camera with a specified number of retries.

    Args:
        max_attempts (int): The maximum number of attempts to take an image.

    Returns:
        np.ndarray: Captured image or None if failed.
    """
    if cam.useCam:  # Check if the camera is enabled
        attempt = 0
        while attempt < max_attempts:
            ok, img, imgTime = cam.getImage()  # Get the image and timestamp
            if ok:
                # If successful, return the image
                return img
            else:
                attempt += 1
                print(f"% Attempt {attempt} failed to get image.")
                if attempt >= max_attempts:
                    print(f"% Failed to get image after {max_attempts} attempts.")
                    return None  # Return None if all attempts fail
    else:
        print("Camera is not in use.")
        return None  # Return None if camera is not enabled


def undistort_image(calibration_file, image):
    """
    Undistorts an image using camera calibration parameters.

    Args:
        calibration_file (str): Path to the .npz file containing calibration data.
        image (str or np.ndarray): Path to the image file or an already loaded image (NumPy array).

    Returns:
        np.ndarray: Undistorted image.
    """
    # Ensure the calibration file is in the same directory as this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_path = os.path.join(script_dir, calibration_file)

    # Load camera calibration data
    with np.load(calibration_path) as data:
        camera_matrix = data["camera_matrix"]
        dist_coeffs = data["dist_coeffs"]

    # Load image if given as a path
    if isinstance(image, str):
        image = cv2.imread(image)
        if image is None:
            raise ValueError(f"❌ Error: Could not load image from {image}")

    # Get image size
    h, w = image.shape[:2]

    # Get optimal camera matrix for better undistortion
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))

    # Undistort the image
    undistorted_img = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)

    # Crop the valid image region (optional)
    x, y, w, h = roi
    if w > 0 and h > 0:
        undistorted_img = undistorted_img[y:y + h, x:x + w]

    return undistorted_img


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
    # Ensure the calibration file is in the same directory as this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_path = os.path.join(script_dir, calibration_file)

    # Load camera calibration data
    with np.load(calibration_path) as data:
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
