# Ohm2017
An obstacle course navigating robot

# Usage
This package targets ROS Kinetic.

Requires [iscumd/SharedROSNodes](https://github.com/iscumd/SharedROSNodes).

# CAMERA CONFIGURATION and SETUP
1. Place rectangular object in front of robot so that center of object is in line with the Lidar's center point
2. Get the coordinates of the rectangle's corners in XY relative to the Lidar center point units in cm
3. Make sure camera mount is secure and will not budge.
4. Take a picture using camera.
5. Open said picture in GIMP and get pixel coordinates of the object starting from top left and working clockwise
6. Input rectangle's real coordinates and pixel coordinates into program (should theorectically be ROS params)
7. 

