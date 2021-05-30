#include "../inc/sumo_main.hpp"
#include "sumo_edge.cpp"
#include "sumo_opponent.cpp"

sumo_main::sumo_main(){}

void sumo_main::run(Mat image){

	// Edge detection
	Point2f robot_position;

	Mat image_gray;
	this->prepare_image(image, image_gray);

	edge_detection.find_robot_position(image_gray);
	robot_position = edge_detection.get_robot_position();


	// Opponent detection
	Point2f opponent_position;

	Mat frame;
	frame = opponent_detection.floor_pixels(image);
	frame = this->get_homography_frame(frame);
	frame = edge_detection.isolate_dohyo(frame);

	opponent_detection.calculate_opponent_position(frame);
	Point2f opp_coord = opponent_detection.get_coordinates();
	opponent_position = edge_detection.calculate_opponent_position(opp_coord);

	//imshow("Dohyo isolated", frame);
	//debug(image);
}

void sumo_main::debug(Mat image){
	Mat lines = edge_detection.draw_lines(image);
	Mat dohyo = edge_detection.draw_dohyo();

	namedWindow("Dohyo", cv::WINDOW_FREERATIO);

	if (opponent_detection.is_opponent_found()){
		Point2f opp  = opponent_detection.get_coordinates();
		double slope = opponent_detection.get_front_slope();

		lines = edge_detection.draw_lines_opponent(lines, opp, slope);
		dohyo = edge_detection.draw_dohyo_opponent(dohyo, opp, slope);
	}

	imshow("Lines", lines);
	imshow("Dohyo", dohyo);
	resizeWindow("Dohyo", 500, 500);
	waitKey(0);
}