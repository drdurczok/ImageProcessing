#include "../inc/image_calibration.hpp"

image_calibration::image_calibration(){
    calibration_file_path = "../calibration/calib.yaml";
	settings_file_path = "../calibration/settings.yaml";

    this->get_settings();
}

// https://learnopencv.com/camera-calibration-using-opencv/

/*
 * Internal Parameters
 * Camera/lens system. E.g. focal length, optical center, and radial distortion coefficients of the lens.
 */

/*
 * External Parameters
 * This refers to the orientation (rotation and translation) of the camera with respect to some world coordinate system.
 */

bool image_calibration::get_settings(){
    //this->remove_file(calibration_file_path);
    //this->remove_file(settings_file_path);

    //Calibration
    if (check_file_exists(calibration_file_path)){
        cout << "Camera configuration file exists, reading parameters" << endl;
    }
    else if (check_images_exist()){
        cout << "Camera configuration does not exist, creating parameters" << endl;
        this->calibrate(); 
    }
    else{
        cout << "Taking images for camera calibration" << endl;
        this->take_calibration_images(); 
        this->calibrate(); 
    }
    this->read_parameters(calibration_file_path);

    //Homography
    if (check_file_exists(settings_file_path)){
        cout << "Camera settings file exists, reading parameters" << endl;
    }
    else if (check_images_exist()){
        cout << "Camera settings don't exist, creating parameters" << endl;
       this->find_homography_matrix();
    }
    else{
        this->take_homography_images();
        this->find_homography_matrix();
    }
    this->read_parameters(settings_file_path);
    this->read_parameters(calibration_file_path);

    return true;
}

void image_calibration::take_calibration_images(){

}

void image_calibration::take_homography_images(){

}


void image_calibration::calibrate(){
    // Creating vector to store vectors of 3D points for each checkerboard image
    vector<vector<Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    vector<vector<Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    vector<Point3f> objp;
    for(int i{0}; i < CHECKERBOARD[1]; i++){
        for(int j{0}; j < CHECKERBOARD[0]; j++)
            objp.push_back(Point3f(j,i,0));
    }


    // Extracting path of individual image stored in a given directory
    vector<String> images;
    // Path of the folder containing checkerboard images
    string path = "../calibration/calib_imgs/*.jpg";

    glob(path, images);

    Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners 
    vector<Point2f> corner_pts;
    bool success;

    // Looping over all the images in the directory
    for(int i{0}; i<images.size(); i++){
        frame = imread(images[i]);
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        success = findChessboardCorners(gray, CHECKERBOARD_SIZE, corner_pts, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

        //this->display_calib_images(frame, corner_pts, to_string(i), success);

        //If desired number of corner are detected, we refine the pixel coordinates and display them on the images of checker board
        if(success){

            TermCriteria criteria(TermCriteria::MAX_ITER | TermCriteria::EPS , 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cornerSubPix(gray,corner_pts, Size(11,11), Size(-1,-1), criteria);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }
        else{
            cout << "Did not detect corners in image number " << i << endl;
        }
    }

    /*
    * Performing camera calibration by passing the value of known 3D points (objpoints)
    * and corresponding pixel coordinates of the detected corners (imgpoints)
    */
    Mat R, T; //3×1 Rotation vector and 3x1 Translation vector
    calibrateCamera(objpoints, imgpoints, Size(gray.rows,gray.cols), this->cameraMatrix, this->distCoeffs, R, T);

    Mat ident = Mat::eye(3,3,CV_32F);

    initUndistortRectifyMap(this->cameraMatrix, this->distCoeffs, ident, this->cameraMatrix, frame.size(), 5, this->mapx, this->mapy);

    //Get image size to save in parameters
    this->IMAGE_SIZE = frame.size();

    this->save_parameters(calibration_file_path);
}

//Replacement for SolvePnP()
void image_calibration::find_homography_matrix(){
    //Get image
    string path = "../calibration/homography.jpg";
    Mat frame = imread(path);


    /*___________________Calculate Real World Points___________________*/

    //Calculate object points
    vector<Point3f> objectPoints;
    for( int i = CHECKERBOARD[1]; i > 0; i-- ){
        for( int j = CHECKERBOARD[0]; j > 0; j-- ){
            objectPoints.push_back(Point3f(float(j*CHECKERBOARD_SQUARE_SIZE),
                                      float(i*CHECKERBOARD_SQUARE_SIZE), 0));
        }
    }

    //The coordinate Z=0 must be removed for the homography estimation part
    vector<Point2f> objectPointsPlanar;
    for (size_t i = 0; i < objectPoints.size(); i++){
        objectPointsPlanar.push_back(Point2f(objectPoints[i].x, objectPoints[i].y));
    }


    /*___________________Calculate Image Points___________________*/

    //Detect chessboard pattern
    vector<Point2f> corners;
    bool found = findChessboardCorners(frame, CHECKERBOARD_SIZE, corners);

    //Display chessboard corners
    //this->display_calib_images(frame, corners, path, found);

    //Undistort points using the intrinsic camera parameters
    vector<Point2f> imagePoints;
    undistortPoints(corners, imagePoints, this->cameraMatrix, this->distCoeffs);
    //TODO currently unused because of normalization of points by undistortPoints causes scaling problems
    //temp code to bypass normalization issue
    imagePoints = corners;
    //end of temp code fix

    /*___________________Calculate Homography___________________*/

    this->homographyMatrix = findHomography(objectPointsPlanar, imagePoints);

    //Normalize matrix
    this->homographyMatrix = this->homographyMatrix / this->homographyMatrix.at<double>(2,2);

    //Calculate the inverse matrix
    this->homographyMatrixInv = this->homographyMatrix.inv();

    /*___________________Calculate Distance to normal___________________*/

    solvePnP(objectPoints, corners, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec);

    //Calculate normal plan
    Mat R;
    Rodrigues(this->rvec, R);
    Mat normal = R*(Mat_<double>(3,1) << 0, 0, 1);

    //Calculate origin of camera on camera plane
    Mat origin(3, 1, CV_64F, Scalar(0));
    origin = R*origin + this->tvec;

    //The distance d can be computed as the dot product between the plane normal and a point on the plane
    this->distanceToPlaneNormal = normal.dot(origin);
    cout << "\ndistanceToPlaneNormal: " << this->distanceToPlaneNormal << endl;

    /*___________________Calculate mm per pixel___________________*/

    Mat homography_frame;
    warpPerspective(frame, homography_frame, this->homographyMatrixInv, frame.size());

    found = findChessboardCorners(homography_frame, CHECKERBOARD_SIZE, corners);
    //this->display_calib_images(homography_frame, corners, "Homography checkerboard", found);

    //TODO use more points to better approximate checkboard_square_size_pixels
    double checkboard_square_size_pixels = sqrt(pow(corners[1].x - corners[0].x, 2) + pow(corners[1].y - corners[0].y, 2) * 1.0); 

    this->pix_to_mm = checkboard_square_size_pixels / CHECKERBOARD_SQUARE_SIZE;

    cout << "Pixels per mm: " << pix_to_mm << endl;

    this->save_parameters(settings_file_path);
}


void image_calibration::save_parameters(string path){
    cv::FileStorage file(path, cv::FileStorage::WRITE);

    if(path == calibration_file_path) {
        file << "imageSize" << this->IMAGE_SIZE;
        file << "cameraMatrix" << this->cameraMatrix;
        file << "distCoeffs" << this->distCoeffs;
        file << "mapx" << this->mapx;
        file << "mapy" << this->mapy;
    }
    else if(path == settings_file_path) {
        file << "rvec" << this->rvec;
        file << "tvec" << this->tvec;
        file << "homographyMatrix" << this->homographyMatrix;
        file << "homographyMatrixInv" << this->homographyMatrixInv;
        file << "distanceToPlaneNormal" << this->distanceToPlaneNormal;
        file << "pix_to_mm" << this->pix_to_mm;
    }

    file.release();

    //this->print_parameters(path);
}

void image_calibration::read_parameters(string path){
    cv::FileStorage file(path, cv::FileStorage::READ);

    if(path == calibration_file_path) {
        file["imageSize"] >> this->IMAGE_SIZE;
        file["cameraMatrix"] >> this->cameraMatrix;
        file["distCoeffs"] >> this->distCoeffs;
        file["mapx"] >> this->mapx;
        file["mapy"] >> this->mapy;
    }
    else if (path == settings_file_path) {
        file["rvec"] >> this->rvec;       
        file["tvec"] >> this->tvec;
        file["homographyMatrix"] >> this->homographyMatrix;
        file["homographyMatrixInv"] >> this->homographyMatrixInv;
        file["distanceToPlaneNormal"] >> this->distanceToPlaneNormal;
        file["pix_to_mm"] >> this->pix_to_mm;
    }

    file.release();

    //this->print_parameters(path);
}

void image_calibration::print_parameters(string path){
    if(path == calibration_file_path) {
        cout << "\nimageSize :\n" << this->IMAGE_SIZE << endl;
        cout << "\ncameraMatrix :\n" << this->cameraMatrix << endl;
        cout << "\ndistCoeffs :\n" << this->distCoeffs << endl;
        //cout << "mapx : " << this->mapx << endl;
        //cout << "mapy : " << this->mapy << endl;
    }
    else if(path == settings_file_path) {
        cout << "\nhomographyMatrix :\n" << this->homographyMatrix << endl;
        cout << "\nhomographyMatrixInv :\n" << this->homographyMatrixInv << endl;
        cout << "\ndistanceToPlaneNormal :\n" << this->distanceToPlaneNormal << endl;
    }
}

inline bool image_calibration::check_file_exists (const std::string& name) {
    struct stat buffer;   
    return (stat (name.c_str(), &buffer) == 0); 
}

inline bool image_calibration::check_images_exist(){
    //TODO check if enough images are taken to create configuration parameters

    return true;
}   

void image_calibration::remove_file(string path){
    if( remove( path.c_str() ) != 0 )
        cout << "Error deleting " << path << endl;
    else
        cout << path << " successfully deleted" << endl;
}

void image_calibration::remove_images(){

}

void image_calibration::display_calib_images(Mat frame, vector<Point2f> corner_pts, string name, bool success){
    // Displaying the detected corner points on the checker board
    drawChessboardCorners(frame, CHECKERBOARD_SIZE, corner_pts, success);
    
    cv::imshow(name, frame);
    cv::waitKey(0);
    
    cv::destroyAllWindows();
}

void image_calibration::get_focal_length_mm(){
    /* Calculates the focal length in mm
     * WARNING: Only may be used if you know what portion of the sensor is being used for the image.
     */

    double fx, fy, f;  // Focal lengths in pixels
    double F;  // Focal length in mm

    fx = this->cameraMatrix.at<double>(0, 0);
    fy = this->cameraMatrix.at<double>(1, 1);

    f = (fx + fy)/2;

    F = f * CAMERA_SENSOR_WIDTH / IMAGE_SIZE.width;

    cout << "\nfocal length in pixels: " << f 
         << "\nfocal length in mm:     " << F << endl;
}

Mat image_calibration::get_camera_position_world_coordinates(){
    Mat R, cameraPosition;

    Rodrigues(this->rvec, R);

    cameraPosition = -R.t() * tvec;

    return cameraPosition;
}

Mat image_calibration::get_world_origin_camera_coordinates(){
    Mat R, worldOrigin;

    Rodrigues(this->rvec, R);

    worldOrigin = R.inv() * (Mat_<double>(3,1) << 0, 0, 1) + this->tvec;

    return worldOrigin;
}
