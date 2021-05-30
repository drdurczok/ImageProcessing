#ifndef SUMO_OPPONENT_H
#define SUMO_OPPONENT_H

#include "image_processing.hpp"


using namespace std;
using namespace cv;


class sumo_opponent : private image_processing {
  public:
    sumo_opponent();

	bool find_opponent_position(Mat);

	bool calculate_opponent_position(Mat);
	Mat floor_pixels(Mat);

	Point2f get_opponent_position();
	double  get_front_slope();
	bool is_opponent_found();

  private:
  	struct position {
      Point2f coordinates;			// With respect to position of self
      double front_slope;
      double length;				// Robot length in mm
	} robot;

	bool success;

    vector<Vec4i> opponent_edge;

  	double calculate_slope(Point2f, Point2f);
  	double calculate_dist(Point2f, Point2f);
  	double calculate_angle(double, double);
  	Point2f calculate_intersection(Point2f, Point2f, Point2f, Point2f);

};

#endif