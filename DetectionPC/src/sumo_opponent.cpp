#include "../inc/sumo_opponent.hpp"

sumo_opponent::sumo_opponent(){}

Point2f sumo_opponent::find_opponent_position(Mat image){
	this->calculate_opponent_position(image);

	return Point2f(0,0);
}

void sumo_opponent::calculate_opponent_position(Mat image){
	Mat HFrame = this->get_homography_frame(image);
	Mat Frame;

	this->prepare_image(HFrame, Frame);

	bool success = false;

    imshow("1", HFrame);
    imshow("2", Frame);


    if (success){
    	cout << "INFO: Locating opponent" << endl;
    }
    else{
		cout << "WARNING: Threshold not found for image." << endl;
	}
}