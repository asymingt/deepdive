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

#include <deepdive.h>

void my_light_process(struct Tracker * tracker, uint32_t timecode,
  uint8_t lh, uint8_t ax, uint8_t sensor, uint16_t angle, uint16_t length) {
  if (lh == 0 && ax == 0)
    printf("[%u][%s] L X SEN (%2u) ANG (%5u) LEN (%5d)\n",
      timecode + angle, tracker->serial, sensor, angle, length);
  if (lh == 0 && ax == 1)
    printf("[%u][%s] L Y SEN (%2u) ANG (%5u) LEN (%5d)\n",
      timecode + angle, tracker->serial, sensor, angle, length);
  if (lh == 1 && ax == 0)
    printf("[%u][%s] R X SEN (%2u) ANG (%5u) LEN (%5d)\n",
      timecode + angle, tracker->serial, sensor, angle, length);
  if (lh == 1 && ax == 1)
    printf("[%u][%s] R Y SEN (%2u) ANG (%5u) LEN (%5d)\n",
      timecode + angle, tracker->serial, sensor, angle, length);
}

void my_imu_process(struct Tracker * tracker, uint32_t timecode,
  int16_t acc[3], int16_t gyr[3], int16_t mag[3]) {
  printf("[%u][%s] I - ACC (%4d,%4d,%4d) GYR (%4d,%4d,%4d)\n", timecode,
    tracker->serial, acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]);
}

void my_but_process(struct Tracker * tracker, uint32_t timecode, uint8_t mask) {
  printf("[%u][%s] B - %u%u%u%u%u%u%u%un", timecode, tracker->serial,
    mask & (1<<7) ? 1 : 0,
    mask & (1<<6) ? 1 : 0,
    mask & (1<<5) ? 1 : 0,
    mask & (1<<4) ? 1 : 0,
    mask & (1<<3) ? 1 : 0,
    mask & (1<<2) ? 1 : 0,
    mask & (1<<1) ? 1 : 0,
    mask & (1<<0) ? 1 : 0);
}

int main() {
  // Initialize the driver
  struct Driver * drv = deepdive_init(0);
  if (!drv) {
    printf("Could not initialize deepdive\n");
    return 1;
  }
  // Install callbacks
  deepdive_install_lig_fn(drv, my_light_process);
  deepdive_install_imu_fn(drv, my_imu_process);
  deepdive_install_but_fn(drv, my_but_process);
  // Keep going until ctrl+c
  while(deepdive_poll(drv) == 0) {}
    return 0;
}

