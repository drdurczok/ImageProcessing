#include "../inc/sumo_opponent.hpp"

sumo_opponent::sumo_opponent(){

}

void sumo_opponent::calculate_opponent_position(Mat image){
	Mat HFrame = img_proc.get_homography_frame(image);
	Mat img_filtered;

	bool success = img_proc.filter(HFrame, img_filtered, 0);

    imshow("1", HFrame);
    imshow("2", img_filtered);


    if (success){
    	cout << "INFO: Locating opponent" << endl;
    }
    else{
		cout << "WARNING: Threshold not found for image." << endl;
	}

}