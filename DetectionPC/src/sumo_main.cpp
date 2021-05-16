#include "../inc/sumo_main.hpp"
#include "sumo_edge.cpp"
#include "sumo_opponent.cpp"

sumo_main::sumo_main(){}

void sumo_main::run(Mat image){
	Point2f robot_position;
	Point2f opponent_position;

	robot_position = edge_detection.find_robot_position(image);
	//opponent_detection.calculate_opponent_position(image);

	//imshow("Dohyo", edge_detection.draw_dohyo());
	//waitKey(0);

}