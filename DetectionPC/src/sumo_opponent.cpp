#include "../inc/sumo_opponent.hpp"

sumo_opponent::sumo_opponent(){}

Point2f sumo_opponent::find_opponent_position(Mat image){
	this->calculate_opponent_position(image);

	return Point2f(0,0);
}

void sumo_opponent::calculate_opponent_position(Mat image){
	bool success = false;

    //imshow("0", image);

    // Copy edges to the images that will display the results in BGR
    //Mat image_copy;
    //cvtColor(image, image_copy, COLOR_GRAY2BGR);

    Mat img_edge = this->find_edge(image, edge_filter_methods::SOBELY);

    //imshow("EDGE", img_edge);

    // Probabilistic Hough Line Transform
    /*
    vector<Vec4i> lines; 										// will hold the results of the detection
    HoughLinesP(img_edge, lines, 1, CV_PI/180, 50, 50, 10 ); 	// runs the actual detection

    // Filter lines
    vector<Vec4i> lines_filtered;
    Vec4i l;
    double len;

    for( size_t i = 0; i < lines.size(); i++ ){
    	l = lines[i];
        len = sqrt((l[0] - l[2])^2 + (l[1] - l[3])^2);

        if (len > 0){
        	lines_filtered.push_back(lines[i]);
        	//cout << "Line length: " << len << endl;
        }
    }*/

	// Draw the lines
    //for( size_t i = 0; i < lines_filtered.size(); i++ ){
    //    l = lines[i];
    //    line( image_copy, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);
    //}

    //cout << "Lines detected: " << lines_filtered.size() << endl;
    //imshow("1", img_edge);
    //imshow("2", image_copy);

    if (success){
    	cout << "INFO: Locating opponent" << endl;
    }
    else{
		//cout << "WARNING: Threshold not found for image." << endl;
	}
}