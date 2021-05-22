#include "image_processing.h"
#include "Arduino.h"

#define WIDTH 160
#define HEIGHT 120
#define ARRAY_SIZE WIDTH*HEIGHT

int16_t linearized_homogeneous_array_X[ARRAY_SIZE];
int16_t linearized_homogeneous_array_Y[ARRAY_SIZE];

bool binFrame[ARRAY_SIZE];
//bool warpedFrame[ARRAY_SIZE];

void get_config(){
  read_homography_map();
}

void print_image_shape(){
  // print shape of image and total length (=heigth*width)
  Serial.print("Width: ");
  Serial.print(frame->width);
  Serial.print("\tHeigth: ");
  Serial.print(frame->height);
  Serial.print("\tLength: ");
  Serial.println(frame->len);
}

void processFrame(){
  threshold();
  warpPerspective();
}

/*
Threshold grayscale image and return a binary image
NOT TESTED
 */
void threshold(){
  uint thresh = 200;
  
  for (uint i=0; i < ARRAY_SIZE; i++){
    if(frame->buf[i] > thresh){
      binFrame[i] = 1;
    }
    else{
      binFrame[i] = 0;
    }
  }
}

/*
Warp perspective based on homography map
NOT TESTED
*/
void warpPerspective(){
  int x, y;
  for (uint i=0; i < ARRAY_SIZE; i++){
    x = linearized_homogeneous_array_X[i];
    y = linearized_homogeneous_array_Y[i];

    if ((x > 0 && x < WIDTH) && (y > 0 && y < HEIGHT)){
      //COMMENTED DUE TO MEMORY ISSUES
      //warpedFrame[i] = binFrame[i];
    }

  }
}


void read_homography_map(){
  // Read parameters from file
  readFile("calibration/homography_map_x.txt", linearized_homogeneous_array_X);
  readFile("calibration/homography_map_y.txt", linearized_homogeneous_array_Y);
}

/*________________________BASE ALGORITHMS________________________*/
// Function that returns dot product of two vector array.
double dotProduct(double vect_A[], double vect_B[]){
  double product = 0;

  int n = 3; //Array size
  for (int i = 0; i < n; i++) 
    product = product + vect_A[i] * vect_B[i];
        
  return product;
}

// Function that returns dot product of two vector array.
void dotProduct(double vect_A[3][3], double vect_B[3], double dot_P[3]){
  dot_P[0] = vect_B[0]*vect_A[0][0] + vect_B[1]*vect_A[0][1] + vect_B[2]*vect_A[0][2];
  dot_P[1] = vect_B[0]*vect_A[1][0] + vect_B[1]*vect_A[1][1] + vect_B[2]*vect_A[1][2];
  dot_P[2] = vect_B[0]*vect_A[2][0] + vect_B[1]*vect_A[2][1] + vect_B[2]*vect_A[2][2];
}

// Function that finds cross product of two vector array.
void crossProduct(double vect_A[], double vect_B[], double cross_P[]){
  cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
  cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
  cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}

void normalize(double vect[3]){
  vect[0] = vect[0] / vect[2];
  vect[1] = vect[1] / vect[2];
  vect[2] = 1;
}
