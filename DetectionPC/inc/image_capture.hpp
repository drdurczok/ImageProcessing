#ifndef IMAGE_CAPTURE_H
#define IMAGE_CAPTURE_H

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

using namespace std;
using namespace cv;

extern string architecture;

class image_capture {
  public:
    image_capture();
    ~image_capture();

    Mat take_image();
    void save_image();

  private:
    VideoCapture cap;

    Mat frame;

    String path_to_saves;
    String img_extension;
    uint16_t img_number;
};

#endif