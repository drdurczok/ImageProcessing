#include "../inc/sumo_main.hpp"
#include "sumo_edge.cpp"
#include "sumo_opponent.cpp"
#include "communications.cpp"

sumo_main::sumo_main(){
	timestamp_start  = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void sumo_main::run(Mat image){
	// Edge detection
	Mat image_gray;
	this->prepare_image(image, image_gray);

	this->robot_success = edge_detection.find_robot_position(image_gray);
	robot_position = edge_detection.get_robot_position();


	// Opponent detection
	Mat frame;
	frame = opponent_detection.floor_pixels(image);
	frame = this->get_homography_frame(frame);
	frame = edge_detection.isolate_dohyo(frame);

	this->opponent_success = opponent_detection.find_opponent_position(frame);
	Point2f opp_coord = opponent_detection.get_opponent_position();
	opponent_position = edge_detection.translate_opponent_position(opp_coord);
	opponent_angle    = atan(-1/opponent_detection.get_front_slope());

	// Prepare message
	if (this->robot_success || this->opponent_success){
		stringstream stream;

		auto timestamp  = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - timestamp_start;
		stream << "T" << timestamp;

		if (this->robot_success){
			stream << fixed << std::setprecision(2) 
				   << "RX" << robot_position.x << "RY" << robot_position.y;
		}
		if (this->opponent_success){
			stream << fixed << std::setprecision(2) 
				   << "OX" << opponent_position.x << "OY" << opponent_position.y 
			   	   << "OA" << opponent_angle;
		}

		string message = stream.str();
		this->comms.send_uart(message);
		cout << message << endl;
	}
	
	this->comms.read_uart();

	#ifdef BENCHMARK
	//imshow("Dohyo isolated", frame);
	debug(image);
	#endif
}



/*______________________VISUAL___________________________*/

void sumo_main::debug(Mat image){
	Mat lines = this->draw_lines(image);
	Mat dohyo = this->draw_dohyo();

	namedWindow("Dohyo", cv::WINDOW_FREERATIO);

	if (opponent_detection.is_opponent_found()){
		Point2f opp  = opponent_detection.get_opponent_position();
		opp = edge_detection.translate_opponent_position(opp);
		double slope = opponent_detection.get_front_slope();

		lines = this->draw_lines_opponent(lines, opp, slope);
		dohyo = this->draw_dohyo_opponent(dohyo, opp, slope);
	}

	imshow("Lines", lines);
	imshow("Dohyo", dohyo);
	resizeWindow("Dohyo", 500, 500);
	waitKey(0);
}

Mat sumo_main::draw_dohyo(){
	string path_to_image = "../calibration/Dohyo.jpg";
    Mat dohyo = imread(path_to_image, IMREAD_COLOR);

    if (this->robot_success){
	    edge_detection.calculate_robot_position();
	   	Point2f robot_position = edge_detection.get_robot_position();
	   	double distance_to_center = edge_detection.get_robot_distance_to_center();
	    Point tangent_point = edge_detection.get_tangent_point();
	    Mat line_tangent = edge_detection.get_line_tangent();
	    Mat line_normal  = edge_detection.get_line_normal();

	    circle(dohyo, robot_position, 20, (0, 0, 255), -1);
		//namedWindow("dohyo", WINDOW_FREERATIO);	
	    //imshow("dohyo", dohyo);
	    //waitKey(0);

	    //cout << "Circle center:            " << this->circle_center << endl;
	    //cout << "Robot Point:              " << robot.coordinates   << endl;	

		//Draw viewing angle lines
		double viewing_angle = 72 * PI/180; //deg to rad
		double view_ang_1 = PI/2 + viewing_angle/2;
		double view_ang_2 = PI/2 - viewing_angle/2;

		vector<Point> line_view_1_points = this->get_fov_line_points(dohyo, robot_position, view_ang_1);
		polylines(dohyo, line_view_1_points, false, Scalar(255, 0, 0), 2, 8);

		vector<Point> line_view_2_points = this->get_fov_line_points(dohyo, robot_position, view_ang_2);
		polylines(dohyo, line_view_2_points, false, Scalar(255, 0, 0), 2, 8);

		ostringstream debug_msg;
		debug_msg << "Tangent: y = " << line_tangent.at<double>(1,0) <<  "x + " << line_tangent.at<double>(0,0);
		Debug(debug_msg.str());
		debug_msg.clear(); debug_msg.str("");

		debug_msg << "Normal: y =  " << line_normal.at<double>(1,0) << "x + " << line_normal.at<double>(0,0);
		Debug(debug_msg.str());
		debug_msg.clear(); debug_msg.str("");

		debug_msg << "Tangent point: " << tangent_point << " [x, y]";
		Debug(debug_msg.str());
		debug_msg.clear(); debug_msg.str("");

		debug_msg << "Robot point:   " << robot_position << " [x, y]";
		Debug(debug_msg.str());

		Debug("Robot distance to center: " + to_string(distance_to_center/10) + " cm");
	}

    return dohyo;
}

Mat sumo_main::draw_dohyo_opponent(Mat input, Point2f opponent_point, double slope){    
    Mat output = input.clone();

    if (this->opponent_success){
		Point2f corner[4];

		corner[0] = opponent_point;
		//circle(output, opponent_point, 20, (0, 0, 255), -1);

	    double side_length_px = 100 / pix_to_mm;

		double b = opponent_point.y - slope * opponent_point.x;
	    double x = opponent_point.x + side_length_px / sqrt(1 + pow(slope,2));
	    double y = x * slope + b;

	    corner[1] = Point(x,y);
	    //circle(frame, Point(x,y), 20, (0, 0, 255), -1);

	    slope = -1/slope;
	    b = opponent_point.y - slope * opponent_point.x;
		x = opponent_point.x - side_length_px / sqrt(1 + pow(slope,2));
	    y = x * slope + b;

	    corner[2] = Point(x,y);
	    //circle(frame, Point(x,y), 20, (0, 0, 255), -1);

	    x = (corner[1].x - corner[0].x) + corner[2].x;
	    y = (corner[1].y - corner[0].y) + corner[2].y;

	    corner[3] = Point(x,y);
	    //circle(frame, Point(x,y), 20, (0, 0, 255), -1);

	    line( output, corner[0], corner[1], Scalar(255), 5, 1 );
	    line( output, corner[0], corner[2], Scalar(255), 5, 1 );
	    line( output, corner[3], corner[1], Scalar(255), 5, 1 );
	    line( output, corner[3], corner[2], Scalar(255), 5, 1 );
    }

    return output;
}

Mat sumo_main::draw_lines(Mat frame){
	if (this->robot_success){
		Point tangent_point = edge_detection.get_tangent_point();
	    Mat line_tangent = edge_detection.get_line_tangent();
	    Mat line_normal  = edge_detection.get_line_normal(); 

		vector<Point> line_tangent_points = this->get_line_points(frame, line_tangent);
		polylines(frame, line_tangent_points, false, Scalar(255, 0, 0), 2, 8);

		vector<Point> line_normal_points = this->get_line_points(frame, line_normal);
		polylines(frame, line_normal_points, false, Scalar(255, 0, 0), 2, 8);

	    circle(frame, tangent_point, 5, (0, 0, 255), -1);
	}

	return frame;
}

Mat sumo_main::draw_lines_opponent(Mat frame, Point2f coord, double slope){
	Point2f corner[4];

	corner[0] = this->homography_calc_inverse(coord);
	//circle(frame, this->homography_calc_inverse(coord), 5, (0, 0, 255), -1);

    double side_length_px = 100 / pix_to_mm;

	double b = coord.y - slope * coord.x;
    double x = coord.x + side_length_px / sqrt(1 + pow(slope,2));
    double y = x * slope + b;

    corner[1] = this->homography_calc_inverse(Point(x,y));
    //circle(frame, this->homography_calc_inverse(Point(x,y)), 5, (0, 0, 255), -1);

    slope = -1/slope;
    b = coord.y - slope * coord.x;
	x = coord.x - side_length_px / sqrt(1 + pow(slope,2));
    y = x * slope + b;

    corner[2] = this->homography_calc_inverse(Point(x,y));
    //circle(frame, this->homography_calc_inverse(Point(x,y)), 5, (0, 0, 255), -1);

    line( frame, corner[0], corner[1], Scalar(255), 2, 1 );
    line( frame, corner[0], corner[2], Scalar(255), 2, 1 );

	return frame;
}

vector<Point> sumo_main::get_fov_line_points(Mat image, Point2f point, double angle){
	Mat line = Mat::zeros(2, 2, CV_64F);
	line.at<double>(1,0) = tan(angle);
	line.at<double>(0,0) = point.y - line.at<double>(1,0) * point.x;

	vector<Point> line_points;
	if (line.at<double>(1,0) < 0){
		for (int x = point.x; x < image.size().width; x++){	// y = b + ax;
			double y = line.at<double>(0, 0) + line.at<double>(1, 0)*x;
			line_points.push_back(Point(x, y));
		}
	}
	else{
		for (int x = point.x; x > 0; x--){					// y = b + ax;
			double y = line.at<double>(0, 0) + line.at<double>(1, 0)*x;
			line_points.push_back(Point(x, y));
		}
	}

	return line_points;
}