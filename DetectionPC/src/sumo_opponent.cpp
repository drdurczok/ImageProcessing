#include "../inc/sumo_opponent.hpp"

sumo_opponent::sumo_opponent(){
    this->robot.length = 20;
}

bool sumo_opponent::find_opponent_position(Mat image){
	bool success = this->calculate_opponent_position(image);

	return success;
}

bool sumo_opponent::calculate_opponent_position(Mat image){
	this->success = false;

    vector<Vec4i> lines;
    const int threshold = 50;
    const double minLineLength = 10;
    const double maxLineGap = 20;
    HoughLinesP( image, lines, 1, CV_PI/180, threshold, minLineLength, maxLineGap );

    /* Draw Lines
    Mat lines_frame = Mat::zeros(image.size().height, image.size().width, CV_8U);
    for( size_t i = 0; i < lines.size(); i++ ){
        line( lines_frame, Point(lines[i][0], lines[i][1]), Point( lines[i][2], lines[i][3]), Scalar(255), 1, 1 );
    }
    imshow("Lines", lines_frame);
    */

    vector<double> slopes;
    for ( auto line : lines){
        slopes.push_back(this->calculate_slope(Point2f(line[0],line[1]), Point2f(line[2],line[3])));
    }

    double slope_inv;
    double tolerance_dist  = 20;            //Pixels
    double tolerance_angle = 20 * PI/180;   //Degrees
    for( size_t i = 0; i < lines.size(); i++ ){
        for( size_t j = i; j < lines.size(); ++j ){
            // Check if lines intersect
            if ( this->calculate_dist(Point2f(lines[i][0],lines[i][1]),Point2f(lines[j][0],lines[j][1])) < tolerance_dist ||
                 this->calculate_dist(Point2f(lines[i][0],lines[i][1]),Point2f(lines[j][2],lines[j][3])) < tolerance_dist ||
                 this->calculate_dist(Point2f(lines[i][2],lines[i][3]),Point2f(lines[j][2],lines[j][3])) < tolerance_dist )
            {
                // Check if slopes are at right angles
                if (abs(this->calculate_angle(slopes[i],slopes[j]) - PI/2) < tolerance_angle) {
                    Point2f pos_of_self = this->get_camera_coordinates();

                    this->opponent_edge.clear();
                    this->opponent_edge.push_back(lines[i]);
                    this->opponent_edge.push_back(lines[j]);
                    
                    this->robot.coordinates = this->calculate_intersection(
                                Point2f(opponent_edge[0][0],opponent_edge[0][1]),
                                Point2f(opponent_edge[0][2],opponent_edge[0][3]),
                                Point2f(opponent_edge[1][0],opponent_edge[1][1]),
                                Point2f(opponent_edge[1][2],opponent_edge[1][3]));
                    this->robot.front_slope  = slopes[i];

                    this->success = true;
                    
                    /* Draw result lines
                    Mat output = Mat::zeros(image.size().height, image.size().width, CV_8U);
                    line( output, Point(lines[i][0], lines[i][1]), Point( lines[i][2], lines[i][3]), Scalar(255), 1, 1 );
                    line( output, Point(lines[j][0], lines[j][1]), Point( lines[j][2], lines[j][3]), Scalar(255), 1, 1 );
                    imshow("Lines Intersection", output);
                    waitKey(0);
                    */                    

                    break;
                } 
            }   
        }
    }



    if (this->success){
    	Debug("Opponent found.");
        return true;
    }
    else{
		CWARN("Threshold not found for image.");
	}

    return false;
}

Point2f sumo_opponent::get_opponent_position(){
    return this->robot.coordinates;
}

double sumo_opponent::get_front_slope(){
    return this->robot.front_slope;
}

bool sumo_opponent::is_opponent_found(){
    return this->success;
}

Mat sumo_opponent::floor_pixels(Mat input){
    cvtColor(input, input, COLOR_RGB2GRAY);

    const int max_lowThreshold = 150;
    const int ratio = 3;
    const int kernel_size = 3;
    Canny( input, input, max_lowThreshold, max_lowThreshold*ratio, kernel_size );

    vector<Point2f> floor_pixels = this->getFloorPixels_Points(input);

    Mat output = Mat::zeros(input.size().height, input.size().width, CV_8U);

    for ( auto pixel : floor_pixels ){
        output.at<uint8_t>(pixel) = 255;
    }

    return output;
}

double sumo_opponent::calculate_slope(Point2f p_1, Point2f p_2){
    if ((p_1.x - p_2.x) != 0){
        return (p_1.y - p_2.y)/(p_1.x - p_2.x);
    }

    return 0;
}

double sumo_opponent::calculate_dist(Point2f p_1, Point2f p_2){
    return sqrt(pow((p_1.y - p_2.y),2)+pow((p_1.x - p_2.x),2));
}

double sumo_opponent::calculate_angle(double slope_1, double slope_2){
    return atan((slope_1 - slope_2)/(1 + slope_1*slope_2));
}

Point2f sumo_opponent::calculate_intersection(Point2f p_1, Point2f p_2, Point2f p_3, Point2f p_4){
    double a1, b1, c1, a2, b2, c2;
    a1 = this->calculate_slope(p_1, p_2);
    a2 = this->calculate_slope(p_3, p_4);
    b1 = -1;
    b2 = -1;
    c1 = p_1.y - a1*p_1.x;
    c2 = p_3.y - a2*p_3.x;

    double x, y;
    x = (b1*c2 - b2*c1)/(a1*b2 - a2*b1);
    y = (c1*a2 - c2*a1)/(a1*b2 - a2*b1);

    return Point2f(x,y);
}