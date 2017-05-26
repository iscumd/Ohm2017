#include <iostream>
#include <stdlib.h>

using namespace std;

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

    cout << "xp1 ";
    cin >> xp1;
    cout << "yp1 ";
    cin >> yp1;
    cout << "xp2 ";
    cin >> xp2;
    cout << "yp2 ";
    cin >> yp2;
    cout << "u1 ";
    cin >> u1;
    cout << "v1 ";
    cin >> v1;
    cout << "u2 ";
    cin >> u2;
    cout << "v2 ";
    cin >> v2;

    a = (u1 - u2) / (xp1 - xp2);
    c = (v1 - v2) / (yp1 - yp2);
    b = u2 - (a * xp2);
    d = v2 - (c * yp2);
    cout << "a " << a << endl;
    cout << "b " << b << endl;
    cout << "c " << c << endl;
    cout << "d " << d << endl;
}
