#ifndef SUMO_ALGO_H
#define SUMO_ALGO_H

using namespace std;
using namespace cv;

class sumo_algo {
  public:
    sumo_algo();

    Mat distance_to_edge(Mat);
    Mat distance_to_center(Mat);

    Mat calculate_ring_center(Mat);

    Mat draw_dohyo();

  private:
    image_processing img_proc;

    struct position {
	  double distance_to_center;
	} robot;


	double outer_ring_diameter_pixels = 1550;
    double outer_ring_radius_pixels = outer_ring_diameter_pixels / 2;
    Point2f circle_center;

    Mat TotalLeastSquares(vector<Point2f>);
    vector<Point> get_line_points(Mat, Mat);

};

#endif