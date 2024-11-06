import cv2
import numpy as np

# Global variables to store selected points
selected_points = []
finished = False

# Mouse callback function for selecting points
def select_points(event, x, y, flags, param):
    global selected_points, finished

    if event == cv2.EVENT_LBUTTONDOWN:
        selected_points.append((x, y))
        if len(selected_points) == 8:
            finished = True


# Function to get color range for the region between two concentric quadrilaterals
def get_color_range(image):
    global selected_points, finished

    # Create a copy of the image for drawing
    img_copy = image.copy()

    # Create a window and set the mouse callback function
    cv2.namedWindow("Select Quadrilaterals")
    cv2.setMouseCallback("Select Quadrilaterals", select_points)

    # Prompt the user to select the corners for the outer and inner quadrilaterals
    print("Select the corners for the outer quadrilateral (in clockwise order):")
    print("Click on four points on the outer quadrilateral.")
    print("Then, select the corners for the inner quadrilateral (in clockwise order):")
    print("Click on four points on the inner quadrilateral.")
    print("Press 'Esc' to exit without selecting.")

    # Loop until the user finishes selecting the corners or presses 'Esc'
    while not finished:
        # Display the image with selected points
        for point in selected_points:
            cv2.circle(img_copy, point, 5, (0, 0, 255), -1)
        cv2.imshow("Select Quadrilaterals", img_copy)

        # Wait for a key press
        key = cv2.waitKey(1) & 0xFF

        # Check if 'Esc' key is pressed
        if key == 27:
            break

    # Check if the user finished selecting the corners
    if finished:
        # Get the selected points for the outer and inner quadrilaterals
        outer_points = np.array(selected_points[:4], dtype=np.float32)
        inner_points = np.array(selected_points[4:], dtype=np.float32)

        # Clear the image copy and selected points
        img_copy = image.copy()
        selected_points = []
        finished = False

        # Draw the selected quadrilaterals on the image
        cv2.polylines(img_copy, [outer_points.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.polylines(img_copy, [inner_points.astype(np.int32)], True, (0, 0, 255), 2)
        cv2.imshow("Select Quadrilaterals", img_copy)

        # Calculate the minimum and maximum x and y coordinates for the selected quadrilaterals
        min_x = int(min(np.min(outer_points[:, 0]), np.min(inner_points[:, 0])))
        max_x = int(max(np.max(outer_points[:, 0]), np.max(inner_points[:, 0])))
        min_y = int(min(np.min(outer_points[:, 1]), np.min(inner_points[:, 1])))
        max_y = int(max(np.max(outer_points[:, 1]), np.max(inner_points[:, 1])))

        # Extract the region between the two quadrilaterals
        region_of_interest = image[min_y:max_y, min_x:max_x]

        # Convert the region to the HSV color space
        hsv_roi = cv2.cvtColor(region_of_interest, cv2.COLOR_BGR2HSV)

        # Calculate the color range for the region
        average_color = np.mean(hsv_roi, axis=(0, 1))
        hue_range = 45  # Adjust this value as needed

        lower_range = np.array([average_color[0] - hue_range, 0, 0])
        upper_range = np.array([average_color[0] + hue_range, 255, 255])

        return lower_range, upper_range

    return None, None


# Load the image
image = cv2.imread(r"C:\Users\81809\Pictures\Camera Roll\WIN_20230524_22_59_24_Pro.jpg")

# Get the color range for the region between two concentric quadrilaterals
lower_range, upper_range = get_color_range(image)

if lower_range is not None and upper_range is not None:
    print("Lower Range: ", lower_range)
    print("Upper Range: ", upper_range)
else:
    print("Quadrilateral selection canceled.")

cv2.waitKey(0)
cv2.destroyAllWindows()
