#include "../inc/image_processing.hpp"

image_processing::image_processing(){
    calibration_file_path = "../calibration/calib.yaml";
	settings_file_path = "../calibration/settings.yaml";

	HCenter = Point2f(0,0);
	radius = 0;

	this->read_camera_parameters(calibration_file_path);
	this->read_camera_parameters(settings_file_path);

	this->calculate_camera_coordinates();
}

Point2f image_processing::getCircleCenter(){
	return HCenter;
}

float image_processing::getCircleRadius(){
	return radius;
}


bool image_processing::filter(Mat input, Mat & output, int selection){
	#ifdef DEBUG
	cout << "INFO: Filtering image." << endl;
	#endif
	if (input.empty()){
		cout << "Error: missing image\n\n" << endl;
	}
	
	//Convert image
	cvtColor(input, output, COLOR_RGB2GRAY);

	//output = this->undistort(output);

	// Get threshold image
	bool success;
	switch (selection){
	    case WHITE:
	    	success = thresh_edge(output, output);
	    	break;
	    default:
	    	success = thresh_edge(output, output);
	}

	if (success) {
		return true;
	}
	else{
		return false;
	}
}

/*------------------------------------------------------------------------------------------*/
Mat image_processing::find_edge(Mat input, edge_filter_methods selection){	
	Mat output;

	switch (selection){
	    case SOBEL:
	    	output = this->sobelEdgeDetection(input);
	    	break;
	    case FLOOR_PIXELS:
	    	output = this->getFloorPixels(input);
	    	break;
	    case CEILING_PIXELS:
	    	output = this->getCeilingPixels(input);
	    	break;
	    default:
	    	output = this->sobelEdgeDetection(input);
	}

	return output;
}

Mat image_processing::sobelEdgeDetection(Mat input){
	Mat sobel, sobelx, sobely;

	/* 
	 * Sobel (input, output, ddepth, dx, dy, ksize=3, scale=1, delta=0, borderType=BORDER_DEFAULT)
	 *		ddepth - output image depth
	 *		dx, dy - order of the derivative
	 *		ksize  - size of the extended Sobel kernel; it must be 1, 3, 5, or 7
	 */

	Sobel(input, sobelx, CV_64F, 1, 0, 5);  // x
	Sobel(input, sobely, CV_64F, 0, 1, 5);  // y

	// Combine images together
	bitwise_and(sobelx, sobely, sobel);

	Mat output;
	sobel.convertTo(output, CV_8UC1, 1);

	return output;
}

/*
 * Copies only first pixel that appears at the bottom of the column
 */
Mat image_processing::getFloorPixels(Mat input){
    Mat bottom_pixels = Mat::zeros(input.rows, input.cols, CV_8UC1);
	
	for (uint i = 0; i < input.cols; i++){
		for(uint j = input.rows-1; j > 0; j--){
			if(input.at<uchar>(j,i) != 0){
				bottom_pixels.at<uchar>(j,i) = 255;
				break;
			}
		}
	}

	return bottom_pixels;
}

/*
 * Copies only first pixel that appears at the top of the column
 */
Mat image_processing::getCeilingPixels(Mat input){
    Mat top_pixels = Mat::zeros(input.rows, input.cols, CV_8UC1);
	
	for (uint i = 0; i < input.cols; i++){
		for(uint j = 0; j < input.rows; j++){
			if(input.at<uchar>(j,i) != 0){
				top_pixels.at<uchar>(j,i) = 255;
				break;
			}
		}
	}

	return top_pixels;
}

/*
 * Copies only first pixel that appears at the bottom of the column as point coord
 */
vector<Point2f> image_processing::getFloorPixels_Points(Mat input){
    vector<Point2f> bottom_pixels;
	
	for (uint i = 0; i < input.cols; i++){
		for(uint j = input.rows-1; j > 0; j--){
			if(input.at<uchar>(j,i) != 0){
				bottom_pixels.push_back(Point2f(i,j));
				break;
			}
		}
	}

	return bottom_pixels;
}


/*
 * Copies only first pixel that appears at the top of the column as point coord
 */
vector<Point2f> image_processing::getCeilingPixels_Points(Mat input){
    vector<Point2f> top_pixels;
	
	for (uint i = 0; i < input.cols; i++){
		for(uint j = 0; j < input.rows; j++){
			if(input.at<uchar>(j,i) != 0){
				top_pixels.push_back(Point2f(i,j));
				break;
			}
		}
	}

	return top_pixels;
}

/*------------------------------------------------------------------------------------------*/
bool image_processing::thresh_edge(Mat input, Mat & result){
	uint min_threshold;
	uint max_threshold = 255;

	Mat kernel_close = getStructuringElement(MORPH_RECT, Size(3,3));
	Mat kernel_open  = getStructuringElement(MORPH_RECT, Size(5,5));

	vector<vector<Point>> contours;
	double area;
	double area_prev = 999999;
	double min_area = 4000;
	double max_area = 20000;
	double max_step_area = 10000;
	double delta_step_area = 0;
	double delta_step_area_prev = 0;

	int starting_thresh = 190;

	Mat output;

	int step = 15;
	for (int min_threshold = starting_thresh; min_threshold > 0 ; min_threshold -= step){
		threshold(input, output, min_threshold, max_threshold, THRESH_BINARY);

		/* Closing morphology, removes small holes */
		morphologyEx(output, output, MORPH_CLOSE, kernel_close);

		/* Opening morphology, removes small objects */
		morphologyEx(output, output, MORPH_OPEN, kernel_open);

		GaussianBlur(output, output, Size(3,3), 1);

		findContours(output, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
		
		/*	Display Contours 
		Mat temp = input.clone();
		drawContours(temp, contours, -1, Scalar(0,255,75), 2);
		imshow("a", temp);
		waitKey(0);
		*/

		if (contours.size() == 1){
			area = contourArea(contours[0]);

			if (area > max_area){break;}
			if (area > min_area){
				if (area - area_prev > max_step_area){break;}
			}

			if (delta_step_area > 0){
				delta_step_area = area - area_prev;
				if (delta_step_area > delta_step_area_prev){break;}
			}
			else{
				delta_step_area = area;
			}

			area_prev = area;
			delta_step_area_prev = delta_step_area;
		}
		else if (contours.size() == 0){
			cout << "INFO: Found no contours" << endl;
		}
		else if (min_threshold == starting_thresh){
			return false;
		}
		else{
			break;
		}

		result = output.clone();
	}

	#ifdef DEBUG
	cout << "Min threshold: " << min_threshold << endl;
	cout << "Area:          " << area_prev << endl;
	#endif

	return true;
}

vector<int> image_processing::calculate_thresholds(Mat1b const& image){
	Mat hist = this->get_histogram(image);

    // Transform Mat to vector
    vector<int> hist_vect;
   	hist_vect.insert(hist_vect.end(), hist.ptr<float>(0), hist.ptr<float>(0)+hist.rows);
  	
  	// Find deviation
	int avg_dev;
	this->findDeviation(hist_vect, avg_dev);

	// Calculate thresholds above average deviation
	int a_prev = 0;
	vector<int> thresh;
   	for (uint i = 0; i < hist_vect.size(); i++){
   		if (abs(a_prev - hist_vect[i]) > avg_dev ){
   			thresh.push_back(i);
   		}
   		a_prev = hist_vect[i];
   	}

	return thresh;   	
}

Mat image_processing::get_histogram(Mat1b const& image){
    // Set histogram bins count
    int bins = 256;
    int histSize[] = {bins};

    // Set ranges for histogram bins
    float lranges[] = {0, 256};
    const float* ranges[] = {lranges};

    // Create matrix for histogram
    Mat hist;
    int channels[] = {0};

    calcHist(&image, 1, channels, Mat(), hist, 1, histSize, ranges, true, false);

    //this->show_histogram(hist, bins);

    return hist;
}

void image_processing::show_histogram(Mat hist, int bins){
	// Visualize each bin
    double max_val=0;
    minMaxLoc(hist, 0, &max_val);

    // Create matrix for histogram visualization
    int const hist_height = 256;
    Mat3b hist_image = Mat3b::zeros(hist_height, bins);

    for(int b = 0; b < bins; b++){
        float const binVal = hist.at<float>(b);
        int   const height = cvRound(binVal*hist_height/max_val);
        line(hist_image, Point(b, hist_height-height), Point(b, hist_height), Scalar::all(255));
    }
    imshow("Histogram", hist_image);
}

// Function to find all the local maxima and minima in the given array arr[] 
void image_processing::findLocalMaximaMinima(int n, vector<int> arr, vector<int> & mx, vector<int> & mn){ 
    // Checking whether the first point is local maxima or minima or none 
    if (arr[0] > arr[1]){
        mx.push_back(0); 
    }
    else if (arr[0] < arr[1]){
        mn.push_back(0); 
    }
  
    // Iterating over all points to check local maxima and local minima 
    for(int i = 1; i < n - 1; i++) { 
	    // Condition for local minima 
	    if ((arr[i - 1] > arr[i]) and (arr[i] < arr[i + 1])){
	        mn.push_back(i); 
	    }
	    // Condition for local maxima 
	    else if ((arr[i - 1] < arr[i]) and (arr[i] > arr[i + 1])){
	        mx.push_back(i); 
	    }
    } 
  
    // Checking whether the last point is local maxima or minima or none 
    if (arr[n - 1] > arr[n - 2]){
        mx.push_back(n - 1); 
    }
    else if (arr[n - 1] < arr[n - 2]){
        mn.push_back(n - 1); 
    }
}

void image_processing::findDeviation(vector<int> arr, int & avg_dev){ 
	avg_dev = 0;

	int a_prev = 0;
	for (int a : arr){
		avg_dev += abs(a_prev - a);
		a_prev = a;
	}

	avg_dev /= arr.size();
}

/*------------------------------------------------------------------------------------------*/
void image_processing::read_camera_parameters(string path){
	cv::FileStorage file(path, cv::FileStorage::READ);
	if (path == calibration_file_path){
		file["cameraMatrix"] >> this->cameraMatrix;
        file["distCoeffs"] >> this->distCoeffs;
	    file["mapx"] >> this->mapx;
	    file["mapy"] >> this->mapy;
	}
	else if (path == settings_file_path){
		file["rvec"] >> this->rvec;
        file["tvec"] >> this->tvec;
        file["homographyMatrix"] >> this->homographyMatrix;
        file["homographyMatrixInv"] >> this->homographyMatrixInv;
        file["distanceToPlaneNormal"] >> this->distanceToPlaneNormal;
        file["pix_to_mm"] >> this->pix_to_mm;
	}
    file.release();
}

Mat image_processing::undistort(Mat img){
	read_camera_parameters(calibration_file_path);

	Mat output;
	remap(img, output, this->mapx, this->mapy, INTER_LINEAR);

	return output;
}

/*------------------------------------------------------------------------------------------*/
Point2f image_processing::homography_calc(Point2f pixel){
    /*
     * Convert point in image frame to homography coordinates.
     *  1 pixel = 1 mm
     */
    //Convert to Mat
    double point_coord[] = {pixel.x, pixel.y, 1};
    Mat CPoint = Mat(3, 1, CV_64F, point_coord);

    //Calculate point in homography frame of reference
    Mat HPoint = this->homographyMatrixInv * CPoint;

    //Noramlize the point
    HPoint =  HPoint / HPoint.at<double>(2);

    //Convert back to Point2f
    return Point2f(HPoint.at<double>(0), HPoint.at<double>(1));
}

Mat image_processing::get_homography_frame(Mat frame){
	Mat homography_frame;

	//Mat large_frame = Mat::zeros(frame.size().height*2, frame.size().width*2, CV_8UC3);
	//frame.copyTo(large_frame(Rect((large_frame.cols - frame.cols)/2, (large_frame.rows - frame.rows)/2, frame.cols, frame.rows)));

    warpPerspective(frame, homography_frame, this->homographyMatrixInv, frame.size());

    return homography_frame;
}

Mat image_processing::get_homography_origin_frame(Mat frame){
	uint CHECKERBOARD_SQUARE_SIZE = 25;
    drawFrameAxes(frame, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec, 2*CHECKERBOARD_SQUARE_SIZE);

    return frame;
}

void image_processing::calculate_camera_coordinates(){
	Mat R, cameraPosition;

    Rodrigues(this->rvec, R);

    cameraPosition = -R.t() * this->tvec;

    this->camera_coordinates = Point2f(cameraPosition.at<double>(0), cameraPosition.at<double>(1));
}

Point2f image_processing::get_camera_coordinates(){
	return this->camera_coordinates;
}


double image_processing::distance_camera_to_pixel(Point2f pixel){
	return sqrt(pow(pixel.x - this->camera_coordinates.x, 2) + pow(pixel.y - this->camera_coordinates.y, 2) * 1.0) * this->pix_to_mm; 
}

Mat image_processing::draw_point_on_frame(Mat frame, Point2f pixel){
	circle(frame, pixel, 6, (0, 0, 255), -1);

	return frame;
}

double image_processing::get_pix_to_mm(){
	return this->pix_to_mm;
}
