#ifndef SUMO_OPPONENT_H
#define SUMO_OPPONENT_H

#include "image_processing.hpp"


using namespace std;
using namespace cv;


class sumo_opponent : private image_processing {
  public:
    sumo_opponent();

	Point2f find_opponent_position(Mat);

	void calculate_opponent_position(Mat);
	Mat floor_pixels(Mat);

  private:
  	struct position {
      Point2f coordinates;
      vector<Vec4i> opponent_edge;
	} robot;

  	double calculate_slope(Point2f, Point2f);
  	double calculate_dist(Point2f, Point2f);
  	double calculate_angle(double, double);
  	Point2f calculate_intersection(Point2f, Point2f, Point2f, Point2f);

};

#endif