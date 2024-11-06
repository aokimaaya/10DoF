import cv2
import numpy as np
import screeninfo

# Global variables
points1 = []
selected_points1 = False


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
    global points1, selected_points1

    if event == cv2.EVENT_LBUTTONDOWN and not selected_points1:
        points1.append((x, y))
        if len(points1) == 4:
            selected_points1 = True
        # Draw a red dot at the clicked position for image 1
        cv2.circle(image1, (x, y), 1, (0, 0, 255), -1)


# Load the first image
image_path = r"C:\Users\81809\Pictures\Camera Roll"
image1 = cv2.imread(
    image_path + r"\2.jpg"
)  # "C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\utils\Before_img_.png")

# Load the second image
# image2 = cv2.imread(r"C:\Users\81809\Pictures\Camera Roll\After_image.jpg")#"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\utils\After_img_.jpg")


# Get screen information
screen = screeninfo.get_monitors()[0]  # Assuming a single monitor setup
screen_width, screen_height = screen.width, screen.height

# Calculate the new size of the image
new_width = int(screen_width * 0.20)
new_height = int(screen_height * 0.50)
new_size = new_width  # , new_height)

## Resize and rotate the first image
image1 = rotate_image(resize_image(image1, new_width), 90)

# Create a window and bind the mouse callback function to it
cv2.namedWindow("Select Quadrilateral 1")
cv2.setMouseCallback("Select Quadrilateral 1", mouse_callback)

# Wait until the quadrilateral is selected for image 1
while not selected_points1:
    cv2.imshow("Select Quadrilateral 1", image1)
    cv2.waitKey(1)

# Save the selected points for future use
points1 = np.float32(points1)

# Perform the perspective transformation for the first image
width = int(
    np.sqrt((points1[2][0] - points1[3][0]) ** 2 + (points1[2][1] - points1[3][1]) ** 2)
)
height = int(
    np.sqrt((points1[1][0] - points1[2][0]) ** 2 + (points1[1][1] - points1[2][1]) ** 2)
)
dst_points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
M1 = cv2.getPerspectiveTransform(points1, dst_points)

# List of image paths to be processed
image_paths = ["2.jpg", "3.jpg", "5.jpg"]

for path in image_paths:
    # Load the image
    image = cv2.imread(image_path + r"\\" + path)

    # Resize and rotate the image
    image = rotate_image(resize_image(image, new_width), 90)

    # Perform the perspective transformation for the image
    cropped_image = cv2.warpPerspective(image, M1, (width, height))

    # Convert the cropped image to grayscale
    gray_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

    # Convert the grayscale image to binary (black and white)
    _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

    # Exclude the red pixels from the black pixel count in the cropped image
    binary_image[np.all(cropped_image == [0, 0, 255], axis=2)] = 255

    # Count the number of black pixels
    black_pixels = np.count_nonzero(binary_image == 0)

    # Display the image, cropped image, binary image, and black pixel count
    cv2.imshow("Image", image)
    cv2.imshow("Cropped Image", cropped_image)
    cv2.imshow("Binary Image", binary_image)
    cv2.putText(
        image,
        f"Black Pixels: {black_pixels}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        2,
    )
    cv2.imshow("Image with Text", image)

    cv2.waitKey(0)

cv2.destroyAllWindows()
