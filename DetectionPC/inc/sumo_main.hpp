#ifndef SUMO_MAIN_H
#define SUMO_MAIN_H

#include "sumo_edge.hpp"
#include "sumo_opponent.hpp"

using namespace std;
using namespace cv;


class sumo_main {
  public:
    sumo_main();

    void run(Mat);

  private:
	sumo_edge     	 edge_detection;
	sumo_opponent 	 opponent_detection;
};

#endif