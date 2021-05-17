#ifndef IMAGE_PROCESSING
#define IMAGE_PROCESSING

#include "esp_camera.h"

extern camera_fb_t * frame;

void get_config();

void print_image_shape();

void homography();

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
