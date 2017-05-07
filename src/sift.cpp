#include "common.h"
#include <opencv2/nonfree/features2d.hpp>
using namespace std;
using namespace cv;

int main(){
  Mat img = imread("/home/shang/dataset/opencv/building.jpg",0);
  if(!img.data){
    cerr << "No data!"<< endl;
    return -1;
  }
  imshow("ORIGIN", img);
  SiftFeatureDetector detector(0,3,0.2,5,1.6);
  vector<KeyPoint> keyPoints;
  detector.detect(img, keyPoints);
  Mat result;
  drawKeypoints(img, keyPoints, result);
//   KeyPoint kp;
//   kp.pt.x
  for(vector<KeyPoint>::const_iterator it=keyPoints.begin(); it!=keyPoints.end(); it++){
    cout << "( " << it->pt.x << ", " << it->pt.y << " )" << endl;
  }
  rectangle(result, Rect(200,200,100,100), Scalar(100,100,100));
  imshow("SIFT keyPoints", result);
  waitKey(0);
}