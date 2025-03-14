import cv2
import numpy as np

def undistort_image(calibration_file, image):
    """
    Undistorts an image using camera calibration parameters.

    Args:
        calibration_file (str): Path to the .npz file containing calibration data.
        image (str or np.ndarray): Path to the image file or an already loaded image (NumPy array).

    Returns:
        np.ndarray: Undistorted image.
    """

    # Load camera calibration data
    with np.load(calibration_file) as data:
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
        undistorted_img = undistorted_img[y:y+h, x:x+w]

    return undistorted_img
