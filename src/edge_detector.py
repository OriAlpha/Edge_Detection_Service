import cv2
import os


def edge_detection(input_image_path, output_image_path):
    '''
    # Edge detection using Gaussian Blur and gradient-based methods

    '''
    # Load the image
    image = cv2.imread(input_image_path)
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply gradient-based edge detection using the Laplacian
    edges = cv2.Laplacian(blurred, cv2.CV_64F)

    # Normalize the edges
    edges = cv2.convertScaleAbs(edges)

    # Make the edges more pronounced by applying thresholding
    _, edges = cv2.threshold(edges, 30, 255, cv2.THRESH_BINARY)

    # Create a mask of green lines
    green_lines = cv2.merge((edges, edges, edges * 0))  # Green lines on a black background

    # Add the green lines to the original image
    result = cv2.addWeighted(image, 1, green_lines, 1, 0)

    # Save the result
    cv2.imwrite(output_image_path, result)

    # Display the result
    cv2.imshow("Edge Detection Result", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Specify the folder containing input images
input_folder = "data_in"

# Create an output folder if it doesn't exist
output_folder = "data_out"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Process each image in the input folder
for filename in os.listdir(input_folder):
    if filename.endswith(".jpg") or filename.endswith(".png"): 
        input_image_path = os.path.join(input_folder, filename)
        output_image_path = os.path.join(output_folder, filename)
        edge_detection(input_image_path, output_image_path)
        print(f"Edge detection completed for {filename}.")

print("All images processed and saved in the output folder.")
