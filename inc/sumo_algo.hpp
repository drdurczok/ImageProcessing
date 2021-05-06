#ifndef SUMO_ALGO_H
#define SUMO_ALGO_H

using namespace std;
using namespace cv;

class sumo_algo {
  public:
    sumo_algo();

    Mat distance_to_edge(Mat);

  private:
    image_processing img_proc;  
};

#endif