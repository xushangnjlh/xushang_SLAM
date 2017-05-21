#include "common.h"

#include <opencv2/xfeatures2d/nonfree.hpp>


using namespace std;
using namespace cv;

Mat origin, img;
vector<Point2f> keyPoints;
string title("Shi_Tomasi");
int maxCorners(10);
int qualityLevel(2);
int minDistance(10);
int blockSize(3);
int useHarrisDetector(0);
int k(4);
int method(1);

void detectCorner(int, void*);

int main(){
  origin = imread("/home/shang/dataset/opencv/building.jpg",CV_LOAD_IMAGE_COLOR);
  if(!origin.data){
    cerr << "No data!"<< endl;
    return -1;
  }
  cvtColor(origin, img, CV_BGR2GRAY);
  namedWindow(title);
  createTrackbar("maxCorners", title, &maxCorners, 200, detectCorner);
  createTrackbar("qualityLevel (%)", title, &qualityLevel, 100, detectCorner);
  createTrackbar("minDistance", title, &minDistance, 400, detectCorner);
  createTrackbar("blockSize", title, &blockSize, 10, detectCorner);
  createTrackbar("useHarrisDetector", title, &useHarrisDetector, 1, detectCorner);
  createTrackbar("k when in HarrisDetector", title, &k, 100, detectCorner);
  createTrackbar("Method", title, &method, 2, detectCorner);
  detectCorner(0,0);
  imshow(title, img);
  
  while(true){
    if(waitKey(0)==27)
      break;
  }
}

void detectCorner(int, void*){
  Mat result ;
  img.copyTo(result);
  switch (method){
    case 1:
      if(qualityLevel==0) {
	qualityLevel = 1;
	setTrackbarPos("qualityLevel (%)", title, 1);
      }
      if(blockSize==0) {
	blockSize = 1;
	setTrackbarPos("blockSize", title, 1);
      }
      goodFeaturesToTrack(result, 
			  keyPoints, 
			  maxCorners, 
			  qualityLevel/100.0, 
			  minDistance, 
			  noArray(), 
			  blockSize,
			  useHarrisDetector,
			  k/100.0
			);
    
      cvtColor(result, result, CV_GRAY2RGB);
      for(vector<Point2f>::const_iterator it=keyPoints.begin(); it!=keyPoints.end(); it++){
	circle(result, *it, 8, Scalar(0,0,255),2);
      }
      imshow(title, result);
      break;
    case 2:
      Ptr<Feature2D> detector = xfeatures2d::SIFT::create();
      vector<KeyPoint> keyPoints;
      detector->detect(result, keyPoints);
      cvtColor(result, result, CV_GRAY2RGB);
      for(vector<KeyPoint>::const_iterator it=keyPoints.begin(); it!=keyPoints.end(); it++){
	rectangle(result, Rect_<float>(it->pt.x-8,it->pt.y-8,16,16), Scalar(0,255,0),2);
      }
      imshow(title, result);
    break;
  }
  
}
