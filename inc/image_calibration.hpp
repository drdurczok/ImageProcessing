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

  private:
  	// Defining the dimensions of checkerboard
	int   CHECKERBOARD[2]{6,9}; 
	float CHECKERBOARD_SQUARE_SIZE = 25.0;
	Size  CHECKERBOARD_SIZE = Size(CHECKERBOARD[0], CHECKERBOARD[1]);

	Mat cameraMatrix;	//Intrinsic Camera Matrix
	Mat distCoeffs;		//Lens distortion coefficients
	Mat R;				//Rotation specified as a 3×1 vector. The direction of the vector specifies the axis of rotation and the magnitude of the vector specifies the angle of rotation.
	Mat T;				//3×1 Translation vector

	Mat newCameraMatrix;
	Mat roi;

	Mat mapx, mapy;

	Mat homographyMatrix;
	double distanceToPlaneNormal;

    string calibration_file_path;
    string settings_file_path;

    void take_calibration_images();
    void take_homography_images();
    void calibrate();
    void find_homography_matrix();

    void save_parameters(string);
    void read_parameters(string);
    void print_parameters(string);
    bool check_file_exists(const std::string&);
    bool check_images_exist();
    void remove_file(string);
    void remove_images();
    void display_calib_images(Mat, vector<Point2f>, bool);

};

#endif