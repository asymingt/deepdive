/* 
  Unofficial driver for Vive Trackers and up to two lighthouses, with an
    emphasis on pulling tracker and lighthouse calibration data from devices.
  
  Adapted from: https://github.com/cnlohr/libsurvive
  Which was based off: https://github.com/collabora/OSVR-Vive-Libre
    Originally Copyright 2016 Philipp Zabel
    Originally Copyright 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
    Originally Copyright (C) 2013 Fredrik Hultin
    Originally Copyright (C) 2013 Jakob Bornecrantz
  Using documentation from: https://github.com/nairol/LighthouseRedox
  
  Copyright (c) 2017 Andrew Symington

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "deepdive_dev_tracker.h"
#include "deepdive_data_imu.h"
#include "deepdive_data_light.h"
#include "deepdive_data_button.h"

// Process light data
void deepdive_dev_tracker_light(struct Tracker * tracker,
  const uint8_t *buf, int32_t len) {
  static uint16_t sensor;
  static uint16_t length;
  static uint32_t timecode;
  for (size_t i = 0; i < 7; i++ ) {
    sensor = *((uint16_t*)(&(buf[i*8+1])));
    length = *((uint16_t*)(&(buf[i*8+3])));
    timecode = *((uint32_t*)(&(buf[i*8+5])));
    if (sensor > 0xfd)
      continue;
    deepdive_data_light(tracker, timecode, sensor, length);
  }
}

// Process IMU data
void deepdive_dev_tracker_imu(struct Tracker * tracker,
  const uint8_t *buf, int32_t len) {
  // Accelerometer (moved from imu-frame to tracker-frame)
  int16_t acc[3], gyr[3];
  acc[0] = *((int16_t*)(buf+1));
  acc[1] = *((int16_t*)(buf+3));
  acc[2] = *((int16_t*)(buf+5));
  gyr[0] = *((int16_t*)(buf+7));
  gyr[1] = *((int16_t*)(buf+9));
  gyr[2] = *((int16_t*)(buf+11));
  // Timecode is wrapping milliseconds
  uint32_t timecode = *((uint32_t*)(buf+13));
  // Process the data
  deepdive_data_imu(tracker, timecode, acc, gyr, NULL);
}

// Process button data
void deepdive_dev_tracker_button(struct Tracker * tracker,
  const uint8_t *buf, int32_t len) {
  uint32_t mask = *((uint32_t*)(buf+7));
  int16_t horizontal = *((int16_t*)(buf+20));
  int16_t vertical = *((int16_t*)(buf+22));
  uint16_t trigger = *((uint16_t*)(buf+26));
  deepdive_data_button(tracker, mask, trigger, horizontal, vertical);
}