# Ohm2017
An obstacle course navigating robot

# Usage
This package targets ROS Kinetic.

Requires [iscumd/SharedROSNodes V1.0.0](https://github.com/iscumd/SharedROSNodes/releases/tag/v1.0.0).

# CAMERA CONFIGURATION and SETUP
## CAMERA MUST BE RECAILBRATED IF POSITION AND ORIENTATION CHANGE
## CAMERA MUST START BEFORE THE MAPPING NODE TO INSURE ADJUSTMENT
1. Place wooden calibration stick named "Ohm Calibration Stick" so that the wooden protrusion that is on the short end of stick is flush with the front of robot and inline with the centerline of the robot.
2.  Place cardboard tri-fold flat with the non-white side flat on the ground and infront of calibration stick so that the center line of tri-fold is inline with the robot's centerline.
3.  From the center of lidar measure the X and Y distances in meters (round to the nearest cm) of 2 diagonal corners i.e O and M
Note. Left of the centerline is negative from the perspective of the robot
                   
                   
           L---------------M
           |               |
           |   tri-fold    |   Height is always perpendicular to the robot
           |               |   Width is always parallel to the robot
           O---------------N
                   |
                   | <-- wooden stick
                   |
                -------
                 robot

4. Open Cheese and change resolution to 640x480, take photo. 
5. Open image in GIMP and get the pixel coodinates of each corner.
6. In vision_test.launch under "Pixel coordinates go here" plug the pixel coordinates into their respective locations


                
          ex. param name="white_line_detection_topLeftXpix" value="247" 

7. Calculate ratio. ratio = Width/Height ( if using the 48x36in tri-fold ratio is 1.3333)
plug the value into the vision_test.launch under "ratio".

8. Now run white_line_detection.cpp using ros, and save the image in the window labeled "WARPED" (there is a save icon) 

9. Open the image labeled "WARPED" and get the new pixel coordinates of the diagonal corners that you found their respective XY distances.

10. To calculate the constants, use either the source "calibration_formula.cpp", or the executable "calibrate" located in Ohm2017/Camera_Calibration, and plug in values for the new pixel coordinates from step 9 and their respective XY distances from step 3.

11. To enable video recording or drawing overlays, in vision_test.launch under "Enable recording" and "Enable drawing" respectively, change false to true. Recording records the raw footage.
                
