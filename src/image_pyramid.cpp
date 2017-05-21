#include <iostream>
using namespace std;

#include "common.h"
#include <opencv2/video/video.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
using namespace cv;

int main ( int argc, char** argv )
{
  Mat src, dst, dst1, dst2;
  src = imread("/home/shang/dataset/opencv/building.jpg");
  if( !src.data ) return -1;
  
  cvtColor(src, src, CV_BGR2GRAY);

//   SiftFeatureDetector detector(0,3,0.1,100,4);
//   vector<KeyPoint> KeyPoints;
//   detector.detect(src, KeyPoints);
//   drawKeypoints(src, KeyPoints, src);
  int ddepth = -1;
  Point anchor = Point( -1, -1 );
  double delta = 0;
  Mat kernel = (Mat_<int>(5,5) << 0,0,1,0,0,0,1,2,1,0,1,2,-16,2,1,0,1,2,1,0,0,0,1,0,0);
  filter2D(src, dst, ddepth , kernel, anchor, delta, BORDER_DEFAULT );
  imshow( "2D LoG", dst );
  
  GaussianBlur(src, dst1, Size(), 0.6);
  GaussianBlur(src, dst2, Size(), 0.8);
  Mat result = dst2 - dst1;
  result.convertTo(result, CV_32F);
  cout << "dbg" << endl;
  imshow("DoG", result);
  waitKey(0);
  return 0;
}