#ifndef IMAGE_CAPTURE
#define IMAGE_CAPTURE

#include "esp_camera.h"

extern camera_fb_t * frame;

void init_camera();
void take_photo();
void save_photo();

#endif
