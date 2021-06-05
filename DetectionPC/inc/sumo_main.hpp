#ifndef SUMO_MAIN_H
#define SUMO_MAIN_H

#include "image_processing.hpp"
#include "sumo_edge.hpp"
#include "sumo_opponent.hpp"
#include "communications.hpp"

using namespace std;
using namespace cv;


class sumo_main : private image_processing {
  public:
    sumo_main();

    void run(Mat);

  private:
	sumo_edge     	 edge_detection;
	sumo_opponent 	 opponent_detection;
	communications 	 comms;

	bool robot_success;
	bool opponent_success;

	Point2f robot_position;
	Point2f opponent_position;
	double  opponent_angle;

	/*______________________VISUAL___________________________*/
	void debug(Mat);
	Mat draw_dohyo();
    Mat draw_dohyo_opponent(Mat, Point2f, double);
    Mat draw_lines(Mat);
    Mat draw_lines_opponent(Mat, Point2f, double);

    vector<Point> get_fov_line_points(Mat, Point2f, double);
};

#endif