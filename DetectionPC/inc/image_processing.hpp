#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cmath>

using namespace std;
using namespace cv;

class image_processing {
  public:
    enum edge_filter_methods{SOBEL,FLOOR_PIXELS,CEILING_PIXELS};

    image_processing();
    Mat undistort(Mat); //TODO: Move to privete after finished testing.

    Mat ellipse_detection(Mat);

    Mat calculate_circle_dimensions(Mat);

    Point2f getCircleCenter();
    float   getCircleRadius();

    //Filters
    Mat filter(Mat);
    Mat find_edge(Mat, edge_filter_methods);
    vector<Point2f> getFloorPixels_Points(Mat);
    vector<Point2f> getCeilingPixels_Points(Mat);

    //Homography
    Point2f homography_calc(Point2f);
    Mat get_homography_frame(Mat);
    Mat get_homography_origin_frame(Mat);
    Point2f get_camera_coordinates();

    //Distance Calculations
    double distance_camera_to_pixel(Point2f);

    Mat draw_point_on_frame(Mat, Point2f);

    Mat distance_to_edge(Mat);

    double get_pix_to_mm();

  private:
    string calibration_file_path;
    string settings_file_path;

    Point2f HCenter;
    float   radius;

    //Filters
    Mat sobelEdgeDetection(Mat);
    Mat getFloorPixels(Mat);
    Mat getCeilingPixels(Mat);

    //Detection
    float verifyCircle(Mat, Point2f, float);
    void getCircle(Point2f&, Point2f&, Point2f&, Point2f&, float&);
    vector<Point2f> getCirclePoints(Mat binaryImage);
    Point2f transform_to_homography_coord(Point2f);

    //Undistortion
    Mat cameraMatrix;
    Mat distCoeffs;     //Lens distortion coefficients
    Mat mapx, mapy;

    void read_camera_parameters(string);

    //Homography
    void calculate_camera_coordinates();

    Mat rvec;
    Mat tvec;
    Mat homographyMatrix;
    Mat homographyMatrixInv;
    double distanceToPlaneNormal;

    //Distance Calculations
    Point2f camera_coordinates;
    double pix_to_mm;

};

#endif