import cv2
import numpy as np

# Global variables to store selected points
outer_points = []
inner_points = []
collecting_outer = True
finished = False

# Mouse callback function for selecting points
def select_points(event, x, y, flags, param):
    global outer_points, inner_points, collecting_outer, finished

    if event == cv2.EVENT_LBUTTONDOWN:
        if collecting_outer:
            outer_points.append((x, y))
        else:
            inner_points.append((x, y))


# Function to draw the polygons on the image
def draw_polygons(image):
    global outer_points, inner_points

    img_copy = image.copy()

    if len(outer_points) > 2:
        cv2.polylines(
            img_copy, [np.array(outer_points, dtype=np.int32)], True, (0, 255, 0), 2
        )

    if len(inner_points) > 2:
        cv2.polylines(
            img_copy, [np.array(inner_points, dtype=np.int32)], True, (0, 0, 255), 2
        )

    return img_copy


# Function to collect points and create polygons
def collect_points(image):
    global collecting_outer, finished

    img_copy = image.copy()

    cv2.namedWindow("Select Polygons")
    cv2.setMouseCallback("Select Polygons", select_points)

    print("Select the corners for the outer polygon.")
    print("Press 'f' to finish selecting outer polygon.")
    print("Select the corners for the inner polygon.")
    print("Press 'f' to finish selecting inner polygon.")
    print("Press 'Esc' to exit without selecting.")

    while not finished:
        img_copy = draw_polygons(img_copy)

        cv2.imshow("Select Polygons", img_copy)

        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # 'Esc' key
            break
        elif key == ord("f"):
            if collecting_outer:
                if len(outer_points) > 2:
                    collecting_outer = False
                    img_copy = draw_polygons(img_copy)
                    cv2.imshow("Select Polygons", img_copy)
                    print("Outer polygon selected.")
            else:
                if len(inner_points) > 2:
                    finished = True

    cv2.destroyAllWindows()


# Function to extract the inner region enclosed by the green tape
def extract_inner_region(image, lower_range, upper_range):
    # Convert the image to the HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create a mask for the color range between the inner and outer polygons
    mask = cv2.inRange(hsv_image, lower_range, upper_range)

    # Apply morphological operations to remove noise and smooth the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Find the contour with the largest area
        largest_contour = max(contours, key=cv2.contourArea)

        # Create a blank mask for the inner region
        inner_mask = np.zeros_like(mask)

        # Draw the largest contour on the inner mask
        cv2.drawContours(inner_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

        # Bitwise AND the image with the inner mask to extract the inner region
        inner_region = cv2.bitwise_and(image, image, mask=inner_mask)

        return inner_region
    else:
        return None


# Load the image
image = cv2.imread(r"C:\Users\81809\Pictures\Camera Roll\WIN_20230524_22_59_24_Pro.jpg")

# Collect points and create polygons
collect_points(image)

# Convert the selected points to NumPy arrays
outer_points = np.array(outer_points, dtype=np.int32)
inner_points = np.array(inner_points, dtype=np.int32)

# Calculate the bounding rectangles for the inner and outer polygons
outer_rect = cv2.boundingRect(outer_points)
inner_rect = cv2.boundingRect(inner_points)

# Extract the region between the inner and outer polygons
region_of_interest = image.copy()
region_of_interest = cv2.rectangle(
    region_of_interest,
    (outer_rect[0], outer_rect[1]),
    (outer_rect[0] + outer_rect[2], outer_rect[1] + outer_rect[3]),
    (0, 255, 0),
    2,
)
region_of_interest = cv2.rectangle(
    region_of_interest,
    (inner_rect[0], inner_rect[1]),
    (inner_rect[0] + inner_rect[2], inner_rect[1] + inner_rect[3]),
    (0, 0, 255),
    2,
)
region_of_interest = region_of_interest[
    outer_rect[1] : outer_rect[1] + outer_rect[3],
    outer_rect[0] : outer_rect[0] + outer_rect[2],
]

# Get the color range from the region between the inner and outer polygons
color_range = cv2.mean(region_of_interest)
lower_range = np.array([color_range[0] - 20, color_range[1] - 50, color_range[2] - 50])
upper_range = np.array([color_range[0] + 20, color_range[1] + 50, color_range[2] + 50])

# Extract the inner region enclosed by the green tape
inner_region = extract_inner_region(image, lower_range, upper_range)

# Check if the inner region extraction was successful
if inner_region is not None:
    # Display the inner region
    cv2.imshow("Inner Region", inner_region)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No inner region found.")
