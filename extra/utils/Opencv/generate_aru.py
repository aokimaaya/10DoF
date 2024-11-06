import os
import cv2
import matplotlib.pyplot as plt
import matplotlib.backends.backend_pdf as pdf_backend

def generate_aruco_markers():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()

    # A4 page size in mm
    page_width = 210
    page_height = 297

    # Calculate the available space for each marker
    margin = 10  # Margin around the markers
    markers_per_row = 3
    markers_per_column = 5
    marker_width = (page_width - 2 * margin) / markers_per_row
    marker_height = (page_height - 2 * margin) / markers_per_column

    # Choose the smaller dimension as the marker size to fit in the available space
    size = int(min(marker_width, marker_height))

    folder_name = "print"

    # Create the "print" folder if it doesn't exist
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    fig = plt.figure(figsize=(page_width / 25.4, page_height / 25.4))
    nx = markers_per_row
    ny = markers_per_column

    pdf_path = os.path.join(folder_name, 'aruco_markers{}x{}.pdf'.format(str(markers_per_row),str(markers_per_column)))
    pdf_pages = pdf_backend.PdfPages(pdf_path)

    marker_ids = list(range(1, 46))  # List of marker IDs
    num_markers = len(marker_ids)
    num_pages = num_markers // (markers_per_row * markers_per_column)
    if num_markers % (markers_per_row * markers_per_column) != 0:
        num_pages += 1

    for page in range(num_pages):
        fig.clear()

        start_idx = page * markers_per_row * markers_per_column
        end_idx = (page + 1) * markers_per_row * markers_per_column

        for i, marker_id in enumerate(marker_ids[start_idx:end_idx]):
            ax = fig.add_subplot(ny, nx, i + 1)
            img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)

            plt.imshow(img, cmap=plt.cm.gray, interpolation="nearest")
            ax.axis("off")

            # Add marker ID as small text at the bottom
            plt.text(0.5, -0.1, str(marker_id), transform=ax.transAxes, fontsize=8, ha='center')

        # Add whitespace below the last row to align the markers
        if i < (markers_per_row * markers_per_column) - 1:
            for _ in range(i + 1, (markers_per_row * markers_per_column)):
                fig.add_subplot(ny, nx, _ + 1).axis("off")

        pdf_pages.savefig(fig, bbox_inches='tight')

    pdf_pages.close()

    plt.close('all')

generate_aruco_markers()
