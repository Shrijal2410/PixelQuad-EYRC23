'''
# Team ID:          2151
# Team Name:        PixelQuad
# Theme:            Luminosity Drone
# Author List:      shrijal2410, rmehta8085, vishalq185m, aniketkumar_8
'''

# Import necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2

# Load the image
image = cv2.imread('led.jpg', cv2.IMREAD_COLOR)

# Convert it to grayscale
grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply Gaussian blur to the grayscale image
blurred_image = cv2.GaussianBlur(grayscale_image, (11, 11), 2)

# Threshold the image to reveal light regions in the blurred image
thresholded_image = cv2.threshold(blurred_image, 200, 255, cv2.THRESH_BINARY)[1]

# Perform a series of erosions and dilations to remove small noise blobs from the thresholded image
thresholded_image = cv2.erode(thresholded_image, None, iterations=2)
thresholded_image = cv2.dilate(thresholded_image, None, iterations=4)

# Perform connected component analysis on the thresholded image
labeled_image = measure.label(thresholded_image, connectivity=2, background=0)
blob_mask = np.zeros(thresholded_image.shape, dtype="uint8")

# Loop over the unique components
for label in np.unique(labeled_image):
    # Ignore the background label
    if label == 0:
        continue

    # Construct the label mask and count the number of pixels
    label_mask = np.zeros(thresholded_image.shape, dtype="uint8")
    label_mask[labeled_image == label] = 255
    num_pixels = cv2.countNonZero(label_mask)

    # Add large blobs to our mask of "large blobs"
    if num_pixels > 300:
        blob_mask = cv2.add(blob_mask, label_mask)

# Find contours in the mask and sort them from left to right
contours_list = cv2.findContours(blob_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours_list = imutils.grab_contours(contours_list)
sorted_contours = contours.sort_contours(contours_list)[0]

# Initialize lists to store centroid coordinates and area
centroid_coordinates = []
area_values = []

# Loop over the contours
for i, contour in enumerate(sorted_contours):
    # Calculate the area of the contour
    contour_area = cv2.contourArea(contour)

    # Draw the bright spot on the image
    cv2.drawContours(image, [contour], -1, (0, 0, 255), 2)

    # Calculate centroid coordinates
    moments = cv2.moments(contour)
    centroid_x = int(moments["m10"] / moments["m00"])
    centroid_y = int(moments["m01"] / moments["m00"])
    centroid = (centroid_x, centroid_y)

    # Append centroid coordinates and area to the respective lists
    centroid_coordinates.append(centroid)
    area_values.append(contour_area)

    # Tag the LED with a label in red color
    cv2.putText(image, f"#{i + 1}", (centroid_x - 10, centroid_y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

# Save the output image as a PNG file
cv2.imwrite("led_detection_results.png", image)

# Open a text file for writing
with open("led_detection_results.txt", "w") as output_file:
    # Write the number of LEDs detected to the file
    output_file.write(f"No. of LEDs detected: {len(sorted_contours)}\n")
    # Loop over the contours
    for i, centroid in enumerate(centroid_coordinates):
        # Write centroid coordinates and area for each LED to the file
        output_file.write(f"Centroid #{i + 1}: {centroid}\nArea #{i + 1}: {area_values[i]}")

# Close the text file
output_file.close()