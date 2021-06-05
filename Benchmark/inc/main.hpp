#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace cv;
using namespace std;

#define PI 3.14159

void run();

void draw_dohyo(Point2f);
vector<Point> get_fov_line_points(Mat, Point2f, double);