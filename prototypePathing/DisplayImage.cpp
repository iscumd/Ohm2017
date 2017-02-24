#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv )
{
    /*if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }*/

    Mat image;
	image.create(100,100, CV_8U);
	
    waitKey(0);

    return 0;
}
