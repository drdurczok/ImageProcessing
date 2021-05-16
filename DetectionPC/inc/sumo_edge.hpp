#ifndef SUMO_EDGE_H
#define SUMO_EDGE_H

#include "image_processing.hpp"

class sumo_edge : private image_processing {
  public:
    sumo_edge();

    Mat distance_to_edge(Mat);
    Mat distance_to_center(Mat);

	Point2f find_robot_position(Mat);

    Point2f calculate_robot_position();
    void    calculate_ring_center(Mat);

	Mat draw_homography_frame(Mat);
    Mat draw_dohyo();

  private:
    struct position {
	  double distance_to_center;
	} robot;

	Mat line_tangent = Mat::zeros(2, 2, CV_64F);
	Mat line_normal  = Mat::zeros(2, 2, CV_64F);
    Point tangent_point;

	double outer_ring_diameter_pixels;
    double outer_ring_radius_pixels;
    Point2f circle_center;

    /*______________________DOHYO CENTER___________________________*/
    void find_tangent(Mat);
    void find_normal();
    void find_circle_center();

    /*______________________THRESHOLD________________________________*/
    bool calculate_threshold_image(Mat, Mat &);

    /*______________________ALGORITHMS_____________________________*/
    Mat TotalLeastSquares(vector<Point2f>);
    vector<Point> get_line_points(Mat, Mat);
    vector<Point> get_fov_line_points(Mat, Point2f, double);

};

#endif