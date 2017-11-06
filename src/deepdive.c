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

// Interface implementations
#include "deepdive_usb.h"

// Initialize the driver
struct Driver * deepdive_init() {
  // Create a new driver context
  struct Driver *drv = malloc(sizeof(struct Driver));
  if (drv == NULL)
    return NULL;
  // General constants
  drv->general.timebase_hz            = 48000000UL; // Ticks per second
  drv->general.timecenter_ticks       = 200000UL;   // Midpoint of sweep
  drv->general.pulsedist_max_ticks    = 500000UL;
  drv->general.pulselength_min_sync   = 2200UL;
  drv->general.pulse_in_clear_time    = 35000UL;
  drv->general.pulse_max_for_sweep    = 1800UL;
  drv->general.pulse_synctime_offset  = 20000UL;
  drv->general.pulse_synctime_slack   = 5000UL;
  // Initialize tracker
  if (deepdive_usb_init(drv) == 0) {
    printf("No devices found\n");
    free(drv);
    return NULL;
  }
  return drv;
}

// Register a light callback function
void deepdive_install_lig_fn(struct Driver * drv,  lig_func fbp) {
  if (drv == NULL) return;
  if (fbp) drv->lig_fn = fbp;
}

// Register an IMU callback function
void deepdive_install_imu_fn(struct Driver * drv,  imu_func fbp) {
  if (drv == NULL) return;
  if (fbp) drv->imu_fn = fbp;
}

// Register an button callback function
void deepdive_install_but_fn(struct Driver * drv, but_func fbp) {
  if (drv == NULL) return;
  if (fbp) drv->but_fn = fbp;
}

// Get the general configuration data
struct General * deepdive_general(struct Driver * drv) {
  if (!drv) return NULL;
  return &drv->general;
}

// Get the calibration data for the lighthouse with the given serial number
struct Lighthouse * deepdive_lighthouse(struct Driver * drv, const char* id) {
  if (!drv) return NULL;
  for (size_t i = 0; i < MAX_NUM_LIGHTHOUSES; i++) {
    if (strcmp(drv->lighthouses[i].serial, id) == 0)
      return &drv->lighthouses[i];
  }
  return NULL;
}

// Get the calibration data for a tracker with the given serial number
struct Tracker * deepdive_tracker(struct Driver * drv, const char* id) {
  if (!drv) return NULL;
  for (size_t i = 0; i < drv->num_trackers; i++) {
    if (strcmp(drv->trackers[i]->serial, id) == 0)
      return drv->trackers[i];
  }
  return NULL;
}

// Poll the driver for events
int deepdive_poll(struct Driver * drv) {
  if (drv == NULL) return -1;
  return libusb_handle_events(drv->usb);
}

// Close the driver and clean up memory
void deepdive_close(struct Driver * drv) {
  if (drv == NULL) return;
  for (size_t i = 0; i < drv->num_trackers; i++) {
    libusb_close(drv->trackers[i]->udev);
    free(drv->trackers[i]);
  }
  free(drv->trackers);
  libusb_exit(drv->usb);
  free(drv);
}
