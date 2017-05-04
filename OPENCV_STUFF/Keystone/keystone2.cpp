#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

Mat Frame, Frame_gray;
Mat dst, detected_edges;

int main( int argc, char** argv )
{
	Mat src = imread("Calibration.jpg");
//	imshow("Original",src);
  		
  			
  	//Compute quad point for edge
    Point Q1=Point2f(557,336);//top left pixel coordinate
    Point Q2=Point2f(832,334);//top right
    Point Q3=Point2f(905,458);//bottom right
    Point Q4=Point2f(512,456);//bottom left

    // compute the size of the card by keeping aspect ratio.
    double ratio=1.505;
    double cardH=sqrt((Q3.x-Q2.x)*(Q3.x-Q2.x)+(Q3.y-Q2.y)*(Q3.y-Q2.y));//Or you can give your own height
    double cardW=ratio*cardH;
    Rect R(Q1.x,Q1.y,cardW,cardH);
	int xshift = 0;
	int yshift = 0;
    Point R1=Point2f(R.x+xshift,R.y+yshift);
    Point R2=Point2f(R.x+R.width+xshift,R.y+yshift);
    Point R3=Point2f(Point2f(R.x+R.width+xshift,R.y+R.height+yshift));
    Point R4=Point2f(Point2f(R.x+xshift,R.y+R.height+yshift));

    std::vector<Point2f> quad_pts;
    std::vector<Point2f> squre_pts;

    quad_pts.push_back(Q1);
    quad_pts.push_back(Q2);
    quad_pts.push_back(Q3);
    quad_pts.push_back(Q4);

    squre_pts.push_back(R1);
    squre_pts.push_back(R2);
    squre_pts.push_back(R3);
    squre_pts.push_back(R4);


    Mat transmtx = getPerspectiveTransform(quad_pts,squre_pts);
    int offsetSize=1000;
    Mat transformed = Mat::zeros(R.height+offsetSize, R.width+offsetSize, CV_8UC3);
    warpPerspective(src, transformed, transmtx, transformed.size());

    //rectangle(src, R, Scalar(0,255,0),1,8,0);

    line(src,Q1,Q2, Scalar(0,0,255),1,CV_AA,0);
    line(src,Q2,Q3, Scalar(0,0,255),1,CV_AA,0);
    line(src,Q3,Q4, Scalar(0,0,255),1,CV_AA,0);
    line(src,Q4,Q1, Scalar(0,0,255),1,CV_AA,0);

    imshow("quadrilateral", transformed);
    imshow("src",src);
    waitKey();
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		
  		waitKey(0);
			
			
		

  return 0;
  }
