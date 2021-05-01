#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <cmath>

using namespace std;
using namespace cv;

#define PI 3.14159265

class image_processing {
  public:
    enum edge_filter_methods{SOBEL,FLOOR_PIXELS,CEILING_PIXELS};

    image_processing();
    Mat undistort(Mat); //TODO: Move to privete after finished testing.
    Mat filter(Mat);
    Mat find_edge(Mat, edge_filter_methods);

    Mat ellipse_detection(Mat, Point2f&, float&);

    Mat position_detection(Mat);

    Point2f getCircleCenter(uint);
    float   getCircleRadius(uint);

    //Pixel to Distance Calculations
    uint pixel_to_distance(uint pixels);

  private:
    string settings_file_path;

    Point2f center[2];
    float   radius[2];

    //Filters
    Mat sobelEdgeDetection(Mat);
    Mat getFloorPixels(Mat);
    Mat getCeilingPixels(Mat);

    //Detection
    float verifyCircle(Mat, Point2f, float);
    void getCircle(Point2f&, Point2f&, Point2f&, Point2f&, float&);
    vector<Point2f> getCirclePoints(Mat binaryImage);

    //Undistortion
    Mat mapx, mapy;

    void read_camera_parameters();

};

#endif