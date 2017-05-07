#include "common.h"

using namespace std;
using namespace cv;

const int median_blur_max = 5;
int median_blur=3;
Mat img;
Mat src, dst;

void callBackFunction( int, void* ) {
    medianBlur(src,dst,median_blur);
    imshow("Controls", img );
}

int main( int argc, char** argv ) {
     namedWindow("Controls", 1);
     createTrackbar("Median blur", "Controls", &median_blur, median_blur_max, callBackFunction );

     callBackFunction(median_blur, &median_blur);

     waitKey(0);
     return 0;
}