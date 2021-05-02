#include "../inc/image_processing.hpp"

image_processing::image_processing(){
    calibration_file_path = "../calibration/calib.yaml";
	settings_file_path = "../calibration/settings.yaml";

	center[0] = Point2f(0,0);
	center[1] = Point2f(0,0);
	radius[0] = 0;
	radius[1] = 0;

	read_camera_parameters(calibration_file_path);
	read_camera_parameters(settings_file_path);
}

Point2f image_processing::getCircleCenter(uint i){
	if (i < sizeof(center)/sizeof(*center)){
		return center[i];
	}
	else{
		cout << "ERROR: Entered index for non-existant circle" << endl;
	}

	return Point2f(0,0);
}

float image_processing::getCircleRadius(uint i){
	if (i < sizeof(radius)/sizeof(*radius)){
		return radius[i];
	}
	else{
		cout << "ERROR: Entered index for non-existant circle" << endl;
	}

	return 0;
}

Mat image_processing::filter(Mat input){
	if (input.empty()){
		cout << "Error: missing image\n\n" << endl;
	}

	//Convert image
	Mat output, kernel;
	cvtColor(input, output, COLOR_RGB2GRAY);

	//output = this->undistort(output);

	/*
	 *	Make binary image
	 *
	 *	threshold(src, dst, min, max)
	 */
	threshold(output, output, 210, 255, THRESH_BINARY);

	/*	
	 *  Closing morphology, removes small holes
	 *	
	 *	Closing = erode(dilute(src, dst, kernel))
	 */
	kernel = getStructuringElement(MORPH_RECT, Size(5,5));
	morphologyEx(output, output, MORPH_CLOSE, kernel);

	/*	
	 *  Opening morphology, removes small objects
	 *	
	 *	Opening = dilate(erode(src, dst, kernel))
	 */
	kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
	morphologyEx(output, output, MORPH_OPEN, kernel);

	/*
	 *	Blurring, smooth the edges
	 *
	 *	GaussianBlur(src, dst, kernel size, sigma)
	 *		sigma - how much image blur
	 */
	GaussianBlur(output, output, Size(3,3), 1);

	return output;
}

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

//_________________________________________________________________
Mat image_processing::position_detection(Mat img){
	Mat mask, circle_bottom, circle_top;
	
	mask = this->find_edge(img, FLOOR_PIXELS);
	circle_bottom = ellipse_detection(mask, center[0], radius[0]);

	mask = this->find_edge(img, CEILING_PIXELS);
	circle_top = ellipse_detection(mask, center[1], radius[1]);

	Mat circle = circle_bottom + circle_top;

	return circle;
}

Mat image_processing::ellipse_detection(Mat mask, Point2f& center, float& radius){
    Mat output = Mat::zeros(mask.rows, mask.cols, CV_8UC1);

	vector<Point2f> edgePositions;
	edgePositions = getCirclePoints(mask);

	// create distance transform to efficiently evaluate distance to nearest edge
	Mat dt;
	distanceTransform(255-mask, dt, DIST_L1, 3);

    // create circle from 3 points:
    getCircle(edgePositions[0],edgePositions[1],edgePositions[2],center,radius);

    float minCirclePercentage = 0.0015f;

    float cPerc = verifyCircle(dt, center, radius);

    if(cPerc >= minCirclePercentage && radius > 6000){
        //cout << "circle with center: " << center << " radius: " << radius << " inliers: " <<  cPerc*100.0f  << "%" << endl;
        circle(output, center, radius, Scalar(255,0,0),1);
	}

	return output;
}

/*
 *  Verifies the circle by counting number of inliers in arc.
 */
float image_processing::verifyCircle(Mat dt, Point2f center, float radius){
	unsigned int counter = 0;
	unsigned int inlier = 0;
	float minInlierDist = 2.0f;
	float maxInlierDistMax = 100.0f;
	float maxInlierDist = radius/25.0f;
	if(maxInlierDist<minInlierDist) maxInlierDist = minInlierDist;
	if(maxInlierDist>maxInlierDistMax) maxInlierDist = maxInlierDistMax;

	// choose samples along the circle and count inlier percentage
	for(float t =0; t<2*3.14159265359f; t+= 0.05f){
		counter++;
		float cX = radius*cos(t) + center.x;
		float cY = radius*sin(t) + center.y;

		if(cX < dt.cols)
		if(cX >= 0)
		if(cY < dt.rows)
		if(cY >= 0)
		if(dt.at<float>(cY,cX) < maxInlierDist){
			inlier++;
		}
	}

	return (float)inlier/float(counter);
}

/*
 *	Calculates center of circle and radius based off 3-points
 */
inline void image_processing::getCircle(Point2f& p1, Point2f& p2, Point2f& p3, Point2f& center, float& radius){
	float x1 = p1.x;
	float x2 = p2.x;
	float x3 = p3.x;

	float y1 = p1.y;
	float y2 = p2.y;
	float y3 = p3.y;

	center.x = (x1*x1+y1*y1)*(y2-y3) + (x2*x2+y2*y2)*(y3-y1) + (x3*x3+y3*y3)*(y1-y2);
	center.x /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

	center.y = (x1*x1 + y1*y1)*(x3-x2) + (x2*x2+y2*y2)*(x1-x3) + (x3*x3 + y3*y3)*(x2-x1);
	center.y /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

	radius = sqrt((center.x-x1)*(center.x-x1) + (center.y-y1)*(center.y-y1));
}


/*
 *	Get three points of arc that will be used to calculate circle
 */
std::vector<cv::Point2f> image_processing::getCirclePoints(Mat binaryImage){
	std::vector<cv::Point2f> pointPositions;

	/*
	 * Use of pointers is fastest, however requires proper range calculations to avoid overflow
	 * "at" does a type check to determine range of type, but is much more resource intensive
	 * Using row pointers avoids the overflow problem with pointers but is much faster than "at"
	 */
	for(unsigned int y = 0; y < binaryImage.rows; ++y){
		unsigned char* rowPtr = binaryImage.ptr<unsigned char>(y);
		
		for(unsigned int x = 0; x < binaryImage.cols; ++x){
			if(rowPtr[x] > 0) pointPositions.push_back(cv::Point2i(x,y));
			//if(binaryImage.at<unsigned char>(y,x) > 0) pointPositions.push_back(Point2f(x,y));
		}
	}

	//TODO improve decision method of choosing arc points
    unsigned int idx1 = rand()%pointPositions.size();
    unsigned int idx2 = rand()%pointPositions.size();
    unsigned int idx3 = rand()%pointPositions.size();

   	std::vector<cv::Point2f> edgePositions;
   	edgePositions.push_back(pointPositions[idx1]);
   	edgePositions.push_back(pointPositions[idx2]);
   	edgePositions.push_back(pointPositions[idx3]);


	return edgePositions;
}

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
	}
    file.release();
}

Mat image_processing::undistort(Mat img){
	read_camera_parameters(calibration_file_path);

	Mat output;
	remap(img, output, this->mapx, this->mapy, INTER_LINEAR);

	// crop the image
	//x, y, w, h = roi;
	//dst = dst[y:y+h, x:x+w]
	//cv.imwrite('calibresult.png', dst)

	return output;
}

uint image_processing::pixel_to_distance(uint pixels){
	/*	Calculates the distance in mm from the number of pixels from the bottom of the screen.
	 *	
	 *	Formula:
	 *		distance = H * tan( min_camera_angle +/- pixels / image_size  * camera_viewing_angle )
	 *			H - height of camera lens
	 *			pixels - number of pixels from bottom of image
	 *			camera_viewing_angle - total viewing angles of camera
	 *
	 *		min_camera_angle = camera_angle - camera_viewing_angle / 2
	 *			camera_angle - angle of camera lens to perpendicular of floor
	 *			
	 */

	float H = 200.0; 					 //[mm]
	float camera_viewing_angle = 40.0; 	 //[deg]
	float image_size = 320.0;
	float camera_angle = 30.0;			 //[deg]

	double min_camera_angle = camera_angle - camera_viewing_angle / 2;

	const double to_rad = PI / 180.0;

	return H * tan( (min_camera_angle + pixels / image_size * camera_viewing_angle) * to_rad );
}

Mat image_processing::homography_calc(Mat pixel){
    /*
     * Convert point in image frame to homogeneous coordinates.
     *  1 pixel = 1 mm
     */
    Mat point_H = this->homographyMatrixInv * pixel;

    //Noramlize the point
    point_H =  point_H / point_H.at<double>(2);

    return point_H;
}

Mat image_processing::get_homography_frame(Mat frame){
	Mat homography_frame;
    warpPerspective(frame, homography_frame, this->homographyMatrixInv, frame.size());

    return homography_frame;
}

Mat image_processing::get_homography_origin_frame(Mat frame){
	uint CHECKERBOARD_SQUARE_SIZE = 25;
    drawFrameAxes(frame, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec, 2*CHECKERBOARD_SQUARE_SIZE);

    return frame;
}