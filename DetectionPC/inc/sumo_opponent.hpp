#ifndef SUMO_OPPONENT_H
#define SUMO_OPPONENT_H

#include "image_processing.hpp"


using namespace std;
using namespace cv;


class sumo_opponent {
  public:
    sumo_opponent();

	void calculate_opponent_position(Mat);

  private:
    image_processing img_proc;

};

#endif