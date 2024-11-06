import cv2
import numpy as np
import screeninfo

# Global variables
points1 = []
points2 = []
selected_points1 = False
selected_points2 = False


def rotate_image(image, angle):
    # Get the image dimensions
    height, width = image.shape[:2]

    # Calculate the rotation matrix
    center = (width // 2, height // 2)
    rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)

    # Calculate the new bounding box dimensions
    cos_theta = np.abs(rotation_matrix[0, 0])
    sin_theta = np.abs(rotation_matrix[0, 1])
    new_width = int((height * sin_theta) + (width * cos_theta))
    new_height = int((height * cos_theta) + (width * sin_theta))

    # Adjust the rotation matrix for translation
    rotation_matrix[0, 2] += (new_width / 2) - center[0]
    rotation_matrix[1, 2] += (new_height / 2) - center[1]

    # Perform the rotation with adjusted dimensions
    rotated_image = cv2.warpAffine(image, rotation_matrix, (new_width, new_height))

    return rotated_image


def resize_image(image, new_width):
    # Get the original image dimensions
    height, width = image.shape[:2]

    # Calculate the scaling factor
    scale = new_width / width

    # Calculate the new height while maintaining the aspect ratio
    new_height = int(height * scale)

    # Resize the image using the calculated dimensions
    resized_image = cv2.resize(image, (new_width, new_height))

    return resized_image


def mouse_callback(event, x, y, flags, param):
    global points1, points2, selected_points1, selected_points2

    if event == cv2.EVENT_LBUTTONDOWN:
        if not selected_points1:
            points1.append((x, y))
            if len(points1) == 4:
                selected_points1 = True
            # Draw a red dot at the clicked position for image 1
            cv2.circle(image1, (x, y), 1, (0, 0, 255), -1)
        elif not selected_points2:
            points2.append((x, y))
            if len(points2) == 4:
                selected_points2 = True
            # Draw a red dot at the clicked position for image 2
            cv2.circle(image2, (x, y), 1, (0, 0, 255), -1)


# Load the first image
image1 = cv2.imread(
    r"C:\Users\81809\Pictures\Camera Roll\Before_image.jpg"
)  # "C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\utils\Before_img_.png")

# Load the second image
image2 = cv2.imread(
    r"C:\Users\81809\Pictures\Camera Roll\After_image.jpg"
)  # "C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\utils\After_img_.jpg")


# Get screen information
screen = screeninfo.get_monitors()[0]  # Assuming a single monitor setup
screen_width, screen_height = screen.width, screen.height

# Calculate the new size of the image
new_width = int(screen_width * 0.50)
# new_height = int(screen_height * 0.50)
# new_size = new_width#, new_height)

# Resize the image
# image1 =rotate_image(image1,90)#resize_image(image1, new_width),90)
image1 = rotate_image(resize_image(image1, new_width), 90)

image2 = rotate_image(resize_image(image2, new_width), 90)

# # Create a window and display the image
# cv2.namedWindow('Resized Image', cv2.WINDOW_NORMAL)
# cv2.imshow('Resized Image', resized_image)
# Create a window and bind the mouse callback function to it
cv2.namedWindow("Select Quadrilateral 1")
cv2.setMouseCallback("Select Quadrilateral 1", mouse_callback)

# Wait until the quadrilateral is selected for image 1
while not selected_points1:
    cv2.imshow("Select Quadrilateral 1", image1)
    cv2.waitKey(1)

# Create a window and bind the mouse callback function to it
cv2.namedWindow("Select Quadrilateral 2")
cv2.setMouseCallback("Select Quadrilateral 2", mouse_callback)

# Wait until the quadrilateral is selected for image 2
while not selected_points2:
    cv2.imshow("Select Quadrilateral 2", image2)
    cv2.waitKey(1)

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

# Apply the perspective transformation to crop the quadrilateral regions for image 1 and image 2
cropped_image1 = cv2.warpPerspective(image1, M1, (width1, height1))
cropped_image2 = cv2.warpPerspective(image2, M2, (width2, height2))

# Convert the cropped images to grayscale for image 1
gray_image1 = cv2.cvtColor(cropped_image1, cv2.COLOR_BGR2GRAY)

# Convert the cropped images to grayscale for image 2
gray_image2 = cv2.cvtColor(cropped_image2, cv2.COLOR_BGR2GRAY)

# Convert the grayscale images to binary (black and white) for image 1
_, binary_image1 = cv2.threshold(gray_image1, 127, 255, cv2.THRESH_BINARY)

# Convert the grayscale images to binary (black and white) for image 2
_, binary_image2 = cv2.threshold(gray_image2, 127, 255, cv2.THRESH_BINARY)

# Exclude the red pixels from the black pixel count in the cropped images
binary_image1[np.all(cropped_image1 == [0, 0, 255], axis=2)] = 255
binary_image2[np.all(cropped_image2 == [0, 0, 255], axis=2)] = 255

# Count the number of black pixels for image 1
black_pixels1 = np.count_nonzero(binary_image1 == 0)

# Count the number of black pixels for image 2
black_pixels2 = np.count_nonzero(binary_image2 == 0)

# Display the images and black pixel counts
cv2.imshow("Image 1", image1)
cv2.imshow("Image 2", image2)
cv2.imshow("Cropped Image 1", cropped_image1)
cv2.imshow("Cropped Image 2", cropped_image2)
cv2.imshow("Binary Image 1", binary_image1)
cv2.imshow("Binary Image 2", binary_image2)
cv2.putText(
    image1,
    f"Black Pixels: {black_pixels1}",
    (10, 30),
    cv2.FONT_HERSHEY_SIMPLEX,
    0.7,
    (0, 0, 255),
    2,
)
cv2.putText(
    image2,
    f"Black Pixels: {black_pixels2}",
    (10, 30),
    cv2.FONT_HERSHEY_SIMPLEX,
    0.7,
    (0, 0, 255),
    2,
)
cv2.imshow("Image 1 with Text", image1)
cv2.imshow("Image 2 with Text", image2)

cv2.waitKey(0)
cv2.destroyAllWindows()

# Print the number of black pixels for image 1
print("Number of black pixels in Image 1:", black_pixels1)

# Print the number of black pixels for image 2
print("Number of black pixels in Image 2:", black_pixels2)
