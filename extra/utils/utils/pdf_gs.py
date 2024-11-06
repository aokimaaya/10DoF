from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas
from PIL import Image

# List of image pairs with names
# image_pairs = 
before_img, after_img= "before_ROI_39_angle35.png"
# PDF filename
pdf_filename = "image_pairs.pdf"

# Create a PDF
c = canvas.Canvas(pdf_filename, pagesize=letter)

# Define the width and height of the images in the PDF (adjust as needed)
image_width = letter[0] / 2
image_height = letter[1] * 0.75

# Iterate through image pairs and add them to the PDF
for before_img, after_img in image_pairs:
    # Open and resize the before and after images
    before = Image.open(before_img)
    before.thumbnail((image_width, image_height))
    after = Image.open(after_img)
    after.thumbnail((image_width, image_height))

    # Calculate the x-coordinate for placing the images side by side
    x = c._pagesize[0] / 4 - image_width / 2

    # Draw the before image
    c.drawImage(before_img, x, 100, width=image_width, height=image_height)

    # Calculate the x-coordinate for the after image
    x = 3 * c._pagesize[0] / 4 - image_width / 2

    # Draw the after image
    c.drawImage(after_img, x, 100, width=image_width, height=image_height)

    # Add a page break
    c.showPage()

# Save the PDF
c.save()

print(f"PDF created: {pdf_filename}")
