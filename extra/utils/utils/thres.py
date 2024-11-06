import cv2
import os
def find_filenames(path_to_dir, suffix=".csv"):
    # path_dir=r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\data\wiping"

    filenames = os.listdir(path_to_dir)
    return [os.path.join(path_to_dir,filename.replace(suffix, ""))
        for filename in filenames
        if filename.endswith(suffix)
    ]

# Function to convert an image to grayscale using adaptive thresholding
def convert_to_grayscale_with_threshold(input_path, output_path):
    # Load the image in color
    image = cv2.imread(input_path)
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply adaptive thresholding
    _, thresholded = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)#cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    
    # Save the resulting image
    cv2.imwrite(output_path, thresholded)
folder_name="2023-10-01_17-23-05.751788"
input_path = r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\data\wiping\{}\train\images_side".format(folder_name)

# List of input folders containing color images
input_folders = find_filenames(input_path,"")#["/path/to/input/folder1", "/path/to/input/folder2"]
print(input_folders)
# Output folder for grayscale images
output_folder = r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\data\wiping\{}\train\images_side\gs".format(folder_name)

# Create the output folder if it doesn't exist
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Iterate over each input folder
for input_folder in input_folders:
    # Traverse through the current input folder
    for root, _, files in os.walk(input_folder):
        print(_,root,files)
        for file in files:
            print(file)
            if file.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp')) and 'ROI' in file and "gs" not in file and "bw" not in file:
                print(file)
                input_path = os.path.join(root, file)
                output_path = os.path.join(output_folder, file)#os.path.join(root, file.replace(".","gs."))
                
                # Convert the image to grayscale with adaptive thresholding and save it
                convert_to_grayscale_with_threshold(input_path, output_path)

print("Conversion complete. Grayscale images are saved in the output folder.")
