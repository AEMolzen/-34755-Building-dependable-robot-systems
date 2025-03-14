import cv2
import numpy as np
import os

# Path to calibration images
image_directory = r"M:\OneDrive DTU\OneDrive - Danmarks Tekniske Universitet\12 Tolvte semester\34755 Building dependable robot systems\34755 camera calibration\calibration_pictures"

# Checkerboard settings
CHECKERBOARD_SIZE = (7, 9)  # Number of inner corners (columns, rows)
SQUARE_SIZE_MM = 20  # Square size in mm
SQUARE_SIZE_M = SQUARE_SIZE_MM / 1000  # Convert to meters

# Termination criteria for refining corner detection
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)

# Prepare object points (3D world coordinates of checkerboard corners)
objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE_M

# Lists to store object and image points
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in the image plane

# Get all image filenames in the directory
image_filenames = [f for f in os.listdir(image_directory) if f.lower().endswith(('.jpg', '.png', '.jpeg'))]

if not image_filenames:
    print("❌ No calibration images found in the directory!")
    exit(1)

for fname in image_filenames:
    img_path = os.path.join(image_directory, fname)
    print(f"📷 Processing: {img_path}")

    # Load image
    img = cv2.imread(img_path)
    if img is None:
        print(f"❌ Error: Could not load {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Optional: Downscale large images to improve processing speed
    if max(gray.shape) > 2000:
        gray = cv2.resize(gray, (gray.shape[1] // 2, gray.shape[0] // 2))
        print("🔻 Image was resized for processing.")

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, None)

    if ret:
        print(f"✅ Chessboard found in {fname}")
        objpoints.append(objp)

        # Refine corner detection
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, CHECKERBOARD_SIZE, corners2, ret)
        cv2.imshow('Calibration Image', img)
        cv2.waitKey(500)
    else:
        print(f"❌ Chessboard NOT found in {fname}")

cv2.destroyAllWindows()

# Ensure we have valid images before calibration
if not objpoints or not imgpoints:
    print("❌ No valid images with detected chessboard corners! Check your images and checkerboard size.")
    exit(1)

# Perform camera calibration
print("\n🔧 Performing camera calibration...")
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

if ret:
    print("✅ Camera calibration successful!")
    print("📌 Camera matrix:\n", camera_matrix)
    print("📌 Distortion coefficients:\n", dist_coeffs)

    # Save calibration results
    np.savez("camera_calibration.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, rvecs=rvecs, tvecs=tvecs)
    print("💾 Calibration data saved to 'camera_calibration.npz'.")
else:
    print("❌ Camera calibration failed. Check input images and chessboard pattern.")

print("🎯 Calibration process completed!")
