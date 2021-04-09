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
	int CHECKERBOARD[2]{6,9}; 

	Mat cameraMatrix, distCoeffs, R, T;
    string settings_file_path;

    void take_calibration_images();
    void calibrate();
    void save_parameters();
    void read_parameters();
    void print_parameters();
    bool check_file_exists(const std::string&);
    bool check_images_exist();
    void remove_calib_file();
    void remove_calib_images();
    void display_calib_images(Mat, vector<Point2f>, bool);

};

#endif