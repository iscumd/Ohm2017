#include <iostream>
#include <stdlib.h>


// this program is used to find the constants for the camera

/*  
LMNO are the corners

                   (xp2, yp2)
                   (u2, v2)
    L-------------M
    |             |
    |             |   Example coordinates       
    |             |
    O-------------N
(xp1, yp1)
(u1, v1)

*/
int main()
{
    int xp1, xp2, yp1, yp2; // pixel coordinates of the 2 corners 
    double u1, u2, v1, v2;  // u's and v's are the X and Y coordinates respectively in meters 
                            //measured from
                            //the from the center of the lidar
                            // if the distance is to the left of lidar it is negative.
    
    double a, b, c, d; // these are the numbers that go into the 
                      // ros params in vision_test.launch under "calibration constants"

    std::cout << "xp1 ";
    std::cin >> xp1;
    std::cout << "yp1 ";
    std::cin >> yp1;
    std::cout << "xp2 ";
    std::cin >> xp2;
    std::cout << "yp2 ";
    std::cin >> yp2;
    std::cout << "u1 ";
    std::cin >> u1;
    std::cout << "v1 ";
    std::cin >> v1;
    std::cout << "u2 ";
    std::cin >> u2;
    std::cout << "v2 ";
    std::cin >> v2;

    a = (u1 - u2) / (xp1 - xp2);
    c = (v1 - v2) / (yp1 - yp2);
    b = u2 - (a * xp2);
    d = v2 - (c * yp2);
    std::cout << "a " << a << std::endl;
    std::cout << "b " << b << std::endl;
    std::cout << "c " << c << std::endl;
    std::cout << "d " << d << std::endl;
}