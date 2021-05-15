#include "../inc/sumo_opponent.hpp"

sumo_opponent::sumo_opponent(){

}

void sumo_opponent::calculate_opponent_position(Mat image){
	Mat HFrame = img_proc.get_homography_frame(image);
	Mat img_filtered = img_proc.filter(HFrame);

    cv::imshow("1", HFrame);
    cv::imshow("2", img_filtered);

}