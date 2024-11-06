import cv2
import cv2.aruco as aruco
import numpy as np

# Set the size of the ChArUco marker in squares
# Adjust the values according to your requirements
squares_x = 5
squares_y = 7
square_length = 100
marker_length = 80

# Generate the ChArUco board
dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length, dictionary)

# Create an image of the ChArUco board
image = board.generateImage((squares_x * square_length, squares_y * square_length))

# Save the ChArUco board image
cv2.imwrite("charuco_marker.png", image)

# Print the ChArUco board information
print("ChArUco board information:")
print(board)

