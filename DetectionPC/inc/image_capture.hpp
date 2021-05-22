#ifndef IMAGE_CAPTURE_H
#define IMAGE_CAPTURE_H

#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

class image_capture {
  public:
    image_capture();

    Mat take_image();
};

#endif