import cv2
import numpy as np

# Global variables
points1 = []
points2 = []
selected_points1 = False
selected_points2 = False


def mouse_callback1(event, x, y, flags, param):
    global points1, selected_points1

    if event == cv2.EVENT_LBUTTONDOWN:
        points1.append((x, y))
        if len(points1) == 4:
            selected_points1 = True

        # Draw a red dot at the clicked position
        cv2.circle(image1, (x, y), 5, (0, 0, 255), -1)


def mouse_callback2(event, x, y, flags, param):
    global points2, selected_points2

    if event == cv2.EVENT_LBUTTONDOWN:
        points2.append((x, y))
        if len(points2) == 4:
            selected_points2 = True

        # Draw a red dot at the clicked position
        cv2.circle(image2, (x, y), 5, (0, 0, 255), -1)


# Load the first image
image1 = cv2.imread(
    r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\utils\Before_img_.png"
)

# Load the second image
image2 = cv2.imread(
    r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\utils\After_img_.jpg"
)

# Create windows for image 1 and image 2
cv2.namedWindow("Image 1")
cv2.namedWindow("Image 2")

# Set mouse callback for image 1
cv2.setMouseCallback("Image 1", mouse_callback1)

# Set mouse callback for image 2
cv2.setMouseCallback("Image 2", mouse_callback2)

while True:
    img1 = image1.copy()
    img2 = image2.copy()

    # Draw circles around the selected corners for image 1
    for i in range(len(points1)):
        cv2.circle(img1, points1[i], 5, (0, 0, 255), 2)

    # Draw circles around the selected corners for image 2
    for i in range(len(points2)):
        cv2.circle(img2, points2[i], 5, (0, 0, 255), 2)

    cv2.imshow("Image 1", img1)
    cv2.imshow("Image 2", img2)
    key = cv2.waitKey(1) & 0xFF

    # Break the loop if 'q' is pressed or the quadrilaterals are selected for both images
    if key == ord("q") or (selected_points1 and selected_points2):
        break

# Sort the points based on x-coordinate to ensure they are in clockwise order for image 1
points1 = sorted(points1, key=lambda x: x[0])

# Sort the points based on x-coordinate to ensure they are in clockwise order for image 2
points2 = sorted(points2, key=lambda x: x[0])

# Rearrange the points in clockwise order starting from the top-left corner for image 1
top_left1 = points1[0] if points1[0][1] < points1[1][1] else points1[1]
top_right1 = points1[1] if points1[0][1] < points1[1][1] else points1[0]
bottom_right1 = points1[2] if points1[2][1] > points1[3][1] else points1[3]
bottom_left1 = points1[3] if points1[2][1] > points1[3][1] else points1[2]

# Rearrange the points in clockwise order starting from the top-left corner for image 2
top_left2 = points2[0] if points2[0][1] < points2[1][1] else points2[1]
top_right2 = points2[1] if points2[0][1] < points2[1][1] else points2[0]
bottom_right2 = points2[2] if points2[2][1] > points2[3][1] else points2[3]
bottom_left2 = points2[3] if points2[2][1] > points2[3][1] else points2[2]

# Calculate the width and height of the quadrilateral for image 1
width1 = int(
    np.sqrt(
        (bottom_right1[0] - bottom_left1[0]) ** 2
        + (bottom_right1[1] - bottom_left1[1]) ** 2
    )
)
height1 = int(
    np.sqrt(
        (top_right1[0] - bottom_right1[0]) ** 2
        + (top_right1[1] - bottom_right1[1]) ** 2
    )
)

# Calculate the width and height of the quadrilateral for image 2
width2 = int(
    np.sqrt(
        (bottom_right2[0] - bottom_left2[0]) ** 2
        + (bottom_right2[1] - bottom_left2[1]) ** 2
    )
)
height2 = int(
    np.sqrt(
        (top_right2[0] - bottom_right2[0]) ** 2
        + (top_right2[1] - bottom_right2[1]) ** 2
    )
)

# Create a perspective transformation matrix for image 1
src_points1 = np.float32([top_left1, top_right1, bottom_right1, bottom_left1])
dst_points1 = np.float32([[0, 0], [width1, 0], [width1, height1], [0, height1]])
M1 = cv2.getPerspectiveTransform(src_points1, dst_points1)

# Create a perspective transformation matrix for image 2
src_points2 = np.float32([top_left2, top_right2, bottom_right2, bottom_left2])
dst_points2 = np.float32([[0, 0], [width2, 0], [width2, height2], [0, height2]])
M2 = cv2.getPerspectiveTransform(src_points2, dst_points2)

# Apply the perspective transformation to crop the quadrilateral regions for image 1
cropped_image1 = cv2.warpPerspective(image1, M1, (width1, height1))

# Apply the perspective transformation to crop the quadrilateral regions for image 2
cropped_image2 = cv2.warpPerspective(image2, M2, (width2, height2))

# Rotate the cropped images by 90 degrees clockwise for image 1
rotated_image1 = cv2.rotate(cropped_image1, cv2.ROTATE_90_CLOCKWISE)

# Rotate the cropped images by 90 degrees clockwise for image 2
rotated_image2 = cv2.rotate(cropped_image2, cv2.ROTATE_90_CLOCKWISE)

# Convert the rotated images to grayscale for image 1
gray_image1 = cv2.cvtColor(rotated_image1, cv2.COLOR_BGR2GRAY)

# Convert the rotated images to grayscale for image 2
gray_image2 = cv2.cvtColor(rotated_image2, cv2.COLOR_BGR2GRAY)

# Threshold the grayscale images to create binary images for image 1
_, binary_image1 = cv2.threshold(gray_image1, 1, 255, cv2.THRESH_BINARY)

# Threshold the grayscale images to create binary images for image 2
_, binary_image2 = cv2.threshold(gray_image2, 1, 255, cv2.THRESH_BINARY)

# Display the original images, cropped images, and binary images side by side
merged_image1 = np.hstack(
    (image1, cropped_image1, cv2.cvtColor(binary_image1, cv2.COLOR_GRAY2BGR))
)
merged_image2 = np.hstack(
    (image2, cropped_image2, cv2.cvtColor(binary_image2, cv2.COLOR_GRAY2BGR))
)

# Add text on the task
cv2.putText(
    merged_image1,
    "Image 1 - Original",
    (10, 30),
    cv2.FONT_HERSHEY_SIMPLEX,
    1,
    (255, 255, 255),
    2,
)
cv2.putText(
    merged_image1,
    "Image 1 - Cropped",
    (520, 30),
    cv2.FONT_HERSHEY_SIMPLEX,
    1,
    (255, 255, 255),
    2,
)
cv2.putText(
    merged_image1,
    "Image 1 - Binary",
    (1030, 30),
    cv2.FONT_HERSHEY_SIMPLEX,
    1,
    (255, 255, 255),
    2,
)

cv2.putText(
    merged_image2,
    "Image 2 - Original",
    (10, 30),
    cv2.FONT_HERSHEY_SIMPLEX,
    1,
    (255, 255, 255),
    2,
)
cv2.putText(
    merged_image2,
    "Image 2 - Cropped",
    (520, 30),
    cv2.FONT_HERSHEY_SIMPLEX,
    1,
    (255, 255, 255),
    2,
)
cv2.putText(
    merged_image2,
    "Image 2 - Binary",
    (1030, 30),
    cv2.FONT_HERSHEY_SIMPLEX,
    1,
    (255, 255, 255),
    2,
)

cv2.imshow("Image 1", merged_image1)
cv2.imshow("Image 2", merged_image2)
cv2.waitKey(0)
cv2.destroyAllWindows()
