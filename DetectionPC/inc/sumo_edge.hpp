#ifndef SUMO_EDGE_H
#define SUMO_EDGE_H

#include "image_processing.hpp"

class sumo_edge : private image_processing {
  public:
    sumo_edge();

    Mat distance_to_edge(Mat);
    Mat distance_to_center(Mat);

	bool   find_robot_position(Mat);
    Point2f get_robot_position();

    void calculate_robot_position();
    bool calculate_ring_center(Mat);
    Point2f translate_opponent_position(Point2f);

    Mat isolate_dohyo(Mat);

	Mat draw_homography_frame(Mat);

    double get_robot_distance_to_center();
    Point get_tangent_point();
    Mat get_line_tangent();
    Mat get_line_normal();

  private:
    struct position {
      double distance_to_center;
      Point2f coordinates;
	} robot;

    bool success;

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

    /*______________________THRESHOLD______________________________*/
    bool calculate_threshold_image(Mat, Mat &);

    /*______________________ALGORITHMS_____________________________*/
    bool check_sagitta_condition(vector<vector<Point>>, vector<vector<Point>> * );

    /*______________________PREDICTIVE_____________________________*/
    vector<Point> assume_dohyo_inner_ring(Mat);
    vector<Point> assume_dohyo_outer_ring(Mat);

};

#endif