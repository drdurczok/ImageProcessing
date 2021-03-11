#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

using namespace std;
using namespace cv;

class image_processing {
  public:
    enum edge_filter_methods{SOBEL};

    image_processing();
    Mat filter(Mat);
    Mat find_edge(Mat, edge_filter_methods);

    Mat ellipse_detection(Mat);

  private:
    Mat sobel_edge_detection(Mat);


};

#endif