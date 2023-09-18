#!/usr/bin/env python

import cv2
import rospy
from srv import EdgeDetection, EdgeDetectionResponse
from msg import EdgePoints

def handle_edge_detection(req):
    input_image_path = req.input_image_path
    output_image_path = req.output_image_path

    # Perform edge detection (you can use the Gaussian Blur and Laplacian method from previous responses)
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

    response = EdgeDetectionResponse()
    response.output_image_path = output_image_path
    return response

def edge_detection_server():
    rospy.init_node('edge_detection_server')
    s = rospy.Service('edge_detection', EdgeDetection, handle_edge_detection)
    rospy.spin()

if __name__ == "__main__":
    edge_detection_server()
