#include "image_capture.h"
#include "sd_card.h"

camera_fb_t * frame;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  init_camera();
  init_sd();
}

void loop() {
  delay(200);
  take_photo();
  save_photo();
}
