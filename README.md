# Edge Detection ROS Service and Client

This package demonstrates a ROS service for performing edge detection on images in a directory using OpenCV. It includes both a server node (`edge_detection_node.py`) for the service and a client node (`edge_detection_client.py`) to request edge detection on images.


1. Build the package using catkin_make:

'''
cd Edge_Detection_Service
catkin_make
'''

2. Source ROS workspace to ensure that ROS can find the package
'''
source devel/setup.bash
'''

3. Create conda env or virtual env for libarary installization 
a. cv2 libary install 

4. Run master node - roscore

5. Run the edge detection server node - rosrun edge_detection_service src/edge_detection_node.py

6. Run the edge detection client node: rosrun edge_detection_service src/edge_detection_client.py