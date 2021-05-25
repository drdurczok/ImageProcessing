#include "../inc/sumo_main.hpp"
#include "sumo_edge.cpp"
#include "sumo_opponent.cpp"

sumo_main::sumo_main(){}

void sumo_main::run(Mat image){
	this->prepare_image(image, image);
	
	Point2f robot_position;
	Point2f opponent_position;

	edge_detection.find_robot_position(image);
	robot_position = edge_detection.get_robot_position();
	opponent_detection.calculate_opponent_position(image);

	debug(image);
}

void sumo_main::debug(Mat image){
	imshow("Lines", edge_detection.draw_lines(image));

	namedWindow("Dohyo", cv::WINDOW_FREERATIO);
	imshow("Dohyo", edge_detection.draw_dohyo());
	resizeWindow("Dohyo", 500, 500);
	waitKey(0);
}