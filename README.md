# HAL Basler Camera (GigE)

HAL for Basler Camera ROS2 Package.

## Description

ROS2 Package to interface to the Basler Camera.
The package depends on Basler SDK (Pylon) and the python wrapper, pyplon (pip install pypylon). 

## Input/Output

Input: 

- rotation_angle: 	angle in degrees for the rotation of the camera feed (double)
- IP 			: 	in case of multiple Basler connected, specify the IP to open a specific camera. No IP means to open the first camera available.

Output: 

- raw_frame: 	camera feed (sensor msgs/Image) 
				published in camera link

## Calibration

Node to calibrate the camera (OpenCv2).

## Depend

- ROS2
- pypylon
- opencv-python
