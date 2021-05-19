#ifndef IMAGE_CALIBRATION_H
#define IMAGE_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h>

using namespace std;
using namespace cv;

class image_calibration {
  public:
    image_calibration();

    bool get_settings();

    void get_focal_length_mm();
    Mat get_camera_position_world_coordinates();
    Mat get_world_origin_camera_coordinates(); 

  private:
  	// Defining the dimensions of checkerboard
	int   CHECKERBOARD[2]{6,8}; 
	float CHECKERBOARD_SQUARE_SIZE = 25.0;
	Size  CHECKERBOARD_SIZE = Size(CHECKERBOARD[0], CHECKERBOARD[1]);

	// Defining physical parameters of camera
	float CAMERA_SENSOR_WIDTH = 3.59;	 // Camera sensor width [mm]
	float CAMERA_SENSOR_HEIGHT = 2.684;  // Camera sensor height [mm]
	Size IMAGE_SIZE;					 // Image size [pixels]
	const double PIXEL_TO_MM = 3.59/320*1.8;

	/* The cameraMatrix is the Intrinsic Camera Matrix, a matrix that contains:
	 * 		-The cameras focal lengths [pixels]
	 *		-The principle points, optical centers [pixels]
	 * Multiply it by a point in the camera coordinate system to obtain a point in
	 * the image coordinate system.
	 */
	Mat cameraMatrix;
	Mat distCoeffs;		//Lens distortion coefficients
	Mat mapx, mapy;

	Mat rvec, tvec;
	Mat homographyMatrix;
	Mat homographyMatrixInv;
	double distanceToPlaneNormal;
	double pix_to_mm;

    string calibration_file_path;
    string settings_file_path;

    void take_calibration_images();
    void take_homography_images();
    void calibrate();
    void find_homography_matrix();
    void calculate_homography_map();
    void create_ring();

    void save_parameters(string);
    void read_parameters(string);
    void print_parameters(string);
    bool check_file_exists(const std::string&);
    bool check_images_exist();
    void remove_file(string);
    void remove_images();
    void display_calib_images(Mat, vector<Point2f>, string, bool);
};

#endif