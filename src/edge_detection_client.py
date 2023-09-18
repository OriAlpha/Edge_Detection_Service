#!/usr/bin/env python

import rospy
from srv import EdgeDetection
from msg import EdgePoints
import os 

def edge_detection_client(input_image_path, output_image_path):
    rospy.wait_for_service('edge_detection')
    try:
        edge_detection = rospy.ServiceProxy('edge_detection', EdgeDetection)
        req = EdgeDetection()
        req.input_image_path = input_image_path
        req.output_image_path = output_image_path
        response = edge_detection(req)
        return response.output_image_path
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))

if __name__ == "__main__":
    rospy.init_node('edge_detection_client')
    input_folder = "data_in"
    output_folder = "data_out"

    for filename in os.listdir(input_folder):
        if filename.endswith(".jpg") or filename.endswith(".png"):
            input_image_path = os.path.join(input_folder, filename)
            output_image_path = os.path.join(output_folder, filename)

            result_image_path = edge_detection_client(input_image_path, output_image_path)

            if result_image_path:
                print(f"Edge detection completed for {filename}. Result saved at {result_image_path}.")
            else:
                print(f"Edge detection failed for {filename}.")

    print("All images processed and saved in the output folder.")
