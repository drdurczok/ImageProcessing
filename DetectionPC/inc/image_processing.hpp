#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cmath>

using namespace std;
using namespace cv;

#define PI 3.14159

class image_processing {
  public:
    enum edge_filter_methods{SOBEL,FLOOR_PIXELS,CEILING_PIXELS};

    image_processing();

    Point2f getCircleCenter();
    float   getCircleRadius();

    void prepare_image(Mat, Mat &);

    /*______________________EDGE DETECTION___________________________*/
    Mat find_edge(Mat, edge_filter_methods);
    vector<Point2f> getFloorPixels_Points(Mat);
    vector<Point2f> getCeilingPixels_Points(Mat);

    /*______________________HOMOGRAPHY_______________________________*/
    Point2f homography_calc(Point2f);
    Mat get_homography_frame(Mat);
    Mat get_homography_frame_from_map(Mat );
    Mat get_homography_origin_frame(Mat);
    Point2f get_camera_coordinates();

    /*______________________DISTANCE_________________________________*/
    double distance_camera_to_pixel(Point2f);
    Mat draw_point_on_frame(Mat, Point2f);
    Mat distance_to_edge(Mat);
    double get_pix_to_mm();

  private:
    string calibration_file_path;
    string settings_file_path;

    Point2f HCenter;
    float   radius;

    /*______________________EDGE DETECTION___________________________*/
    Mat sobelEdgeDetection(Mat);
    Mat getFloorPixels(Mat);
    Mat getCeilingPixels(Mat);

    /*______________________CAMERA___________________________________*/
    Mat cameraMatrix;
    Mat distCoeffs;     //Lens distortion coefficients
    Mat mapx, mapy;

    void read_camera_parameters(string);
    void read_file(string, int16_t array[]);
    Mat undistort(Mat);

    /*______________________HOMOGRAPHY_______________________________*/
    void calculate_camera_coordinates();

    Mat rvec;
    Mat tvec;
    Mat homographyMatrix;
    Mat homographyMatrixInv;

    uint16_t rows = 320;
    uint16_t cols = 240;
    uint32_t array_size = cols*rows;
    int16_t linearized_homogeneous_array[2][320*240];

    double distanceToPlaneNormal;

    /*______________________DISTANCE_________________________________*/
    Point2f camera_coordinates;
    double pix_to_mm;

};

#endif