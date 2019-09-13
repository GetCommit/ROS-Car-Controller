

/color_tracking/
 — src/color_tracking_node.cpp

** Goal:
	This folder is the ROS package modified from /sample_subpub_img/ sample codes. “color_tracking_node.cpp” can subscribe ZED RGB topic, “rgb/image_rect”, and then filter out colors except blue inside cv_color_tracking() function. It can also do the edge detection inside cv_process_img(). cv_publish_img() publishes the results from edge detection.

	- This code can directly run on the car. You can use HDMI line to connect the car to external display to visualize the result images, both images from edge detection and from color tracking.

	- Since it publish edge detection image to ROS, you can also subscribe this image from your remote computer.


** How to run:
	- After you catkin_make this package
	- roscoe
	- rosrun zed_wrapper zed_wrapper_node
	- rosrun color_tracking color_tracking_node

** References:
	- http://opencv-srf.blogspot.com/2010/09/object-detection-using-color-seperation.html
	- http://aishack.in/tutorials/tracking-colored-objects-opencv/
	- https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
 
** Blue detection demo video:
	https://www.youtube.com/watch?v=XBJ6gpdSpMU&feature=youtu.be




 