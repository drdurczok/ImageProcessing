#ifndef IMAGE_PROCESSING
#define IMAGE_PROCESSING

#include <vector>
#include "esp_camera.h"
#include "sd_card.h"

extern camera_fb_t * frame;

void get_config();

void print_image_shape();

void processFrame();

void threshold();
void warpPerspective();
void read_homography_map();

double dotProduct(double vect_A[], double vect_B[]);
void dotProduct(double vect_A[3][3], double vect_B[3], double dot_P[3]);
void crossProduct(double vect_A[], double vect_B[], double cross_P[]);

void normalize(double vect[3]);

#endif


/*DOCUMENTATION SHORTCUTS*/
/*
typedef struct {
    uint8_t * buf;              //!< Pointer to the pixel data
    size_t len;                 //!< Length of the buffer in bytes
    size_t width;               //!< Width of the buffer in pixels
    size_t height;              //!< Height of the buffer in pixels
    pixformat_t format;         //!< Format of the pixel data
} camera_fb_t;
*/
