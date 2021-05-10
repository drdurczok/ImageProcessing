#ifndef SUMO_ALGO_H
#define SUMO_ALGO_H

using namespace std;
using namespace cv;

#define PI 3.14159

class sumo_algo {
  public:
    sumo_algo();

    Mat distance_to_edge(Mat);
    Mat distance_to_center(Mat);

	Point2f find_robot_position(Mat);

    void    calculate_ring_center(Mat);
    Point2f calculate_robot_position();

	Mat draw_homography_frame(Mat);
    Mat draw_dohyo();

  private:
    image_processing img_proc;

    struct position {
	  double distance_to_center;
	} robot;

	Mat line_tangent = Mat::zeros(2, 2, CV_64F);
	Mat line_normal  = Mat::zeros(2, 2, CV_64F);

	double outer_ring_diameter_pixels = 1550;
    double outer_ring_radius_pixels = outer_ring_diameter_pixels / 2;
    Point2f circle_center;

    Mat TotalLeastSquares(vector<Point2f>);
    vector<Point> get_line_points(Mat, Mat);
    vector<Point> get_fov_line_points(Mat, Point2f, double);


};

#endif