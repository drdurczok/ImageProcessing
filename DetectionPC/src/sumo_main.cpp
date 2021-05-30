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

	//Mat image_inv, image_gray_inv;
	//bitwise_not(image, image_inv);	// Invert Image
	//this->prepare_image(image_inv, image_gray_inv);


	Mat frame;
	frame = opponent_detection.floor_pixels(image);
	frame = this->get_homography_frame(frame);
	frame = edge_detection.isolate_dohyo(frame);

	opponent_detection.calculate_opponent_position(frame);


	//imshow("Dohyo isolated", dohyo_img);
	//debug(image);
}

void sumo_main::debug(Mat image){
	imshow("Lines", edge_detection.draw_lines(image));

	namedWindow("Dohyo", cv::WINDOW_FREERATIO);
	imshow("Dohyo", edge_detection.draw_dohyo());
	resizeWindow("Dohyo", 500, 500);
	waitKey(0);
}