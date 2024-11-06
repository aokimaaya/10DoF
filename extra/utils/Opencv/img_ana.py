import cv2
import numpy as np

# Load the image
image = cv2.imread(r"C:\Users\81809\Pictures\Camera Roll\WIN_20230524_22_59_24_Pro.jpg")

# Convert the image to the HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define the lower and upper bounds for the green color
lower_green = np.array([40, 0, 0])  # Adjust these values as needed
upper_green = np.array([81, 255, 255])  # Adjust these values as needed

# Create a mask for the green color
mask = cv2.inRange(hsv, lower_green, upper_green)

# Apply morphological operations to remove noise
kernel = np.ones((5, 5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# Find contours in the mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Initialize a blank mask to draw the detected region
region_mask = np.zeros_like(mask)

# Find the largest contour
if len(contours) > 0:
    largest_contour = max(contours, key=cv2.contourArea)

    # Approximate the contour as a polygon
    epsilon = 0.1 * cv2.arcLength(largest_contour, True)
    approx_polygon = cv2.approxPolyDP(largest_contour, epsilon, True)

    # Draw the polygon on the region mask
    cv2.drawContours(region_mask, [approx_polygon], 0, (255), cv2.FILLED)

# Apply the mask to the original image
result = cv2.bitwise_and(image, image, mask=region_mask)

# Display the result
cv2.imshow("Result", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
