#ifndef IMAGE_CORE_H
#define IMAGE_CORE_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

class image_core {
  public:
  	image_core();
  	cv::Mat get_image();
};

#endif