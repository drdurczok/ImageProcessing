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

  private:

};

#endif