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

  private:
    image_processing img_proc;

    Mat TotalLeastSquares(vector<Point2f>);
    vector<Point> get_line_points(Mat, Mat);

};

#endif