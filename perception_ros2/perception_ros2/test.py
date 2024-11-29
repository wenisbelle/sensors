import cv2
import numpy as np

# Create a blank white image
image = 255 * np.ones(shape=[500, 500, 3], dtype=np.uint8)

# Set grid spacing and point radius
spacing = 50  # Distance between points
radius = 5    # Radius of each point

# Define some colors for variety
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 0, 255), (255, 255, 0)]

# Draw a grid of points on the image
for i in range(radius, image.shape[0], spacing):
    for j in range(radius, image.shape[1], spacing):
        color = colors[(i // spacing + j // spacing) % len(colors)]  # Cycle through colors
        cv2.circle(image, (j, i), radius, color, -1)  # Draw filled circles

# Display the image
cv2.imshow("Grid of Points", image)
cv2.waitKey(0)  # Wait until a key is pressed
cv2.destroyAllWindows()
