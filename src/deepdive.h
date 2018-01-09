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

#ifndef LIBDEEPDIVE_DEEPDIVE_H
#define LIBDEEPDIVE_DEEPDIVE_H

#include <libusb-1.0/libusb.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// Uncomment the line below if you want the IMU in the tracker
// frame (this might affect scale and bias parameters). By
// default we keep this in the IMU frame.
// #define IMU_FRAME_TRACKER

#define MAX_PACKET_LEN        64
#define PREAMBLE_LENGTH       17

#define MAX_NUM_LIGHTHOUSES   2
#define MAX_NUM_SENSORS       32
#define MAX_SERIAL_LENGTH     32
#define USB_INT_BUFF_LENGTH   64

#define USB_VEND_HTC          0x28de
#define USB_PROD_TRACKER      0x2022
#define USB_PROD_WATCHMAN     0x2101

#define USB_ENDPOINT_GENERAL  0x81
#define USB_ENDPOINT_LIGHT    0x82
#define MAX_ENDPOINTS         2

#define DEFAULT_ACC_SCALE     (float)(9.80665/4096.0)
#define DEFAULT_GYR_SCALE     (float)((1./32.768)*(3.14159/180.));

// Forward declaration of driver context
struct Driver;
struct Tracker;

// Extrinsics axes
typedef enum {
  TRACKER_IMU   = 0,
  TRACKER_LIGHT = 1,
  WATCHMAN      = 2
} CallbackType;

// Interrupt buffer for an endpoint
struct Endpoint {
  struct Tracker *tracker;
  CallbackType type;
  struct libusb_transfer *tx;
  uint8_t buffer[USB_INT_BUFF_LENGTH];
};

// Calibration for a given tracker
struct Calibration {
  uint32_t timestamp;                       // Time of last update
  uint8_t num_channels;                     // Number of photodiodes (PDs)
  uint8_t channels[MAX_NUM_SENSORS];        // Channel assignment for PDs
  float positions[MAX_NUM_SENSORS][3];      // PD positions
  float normals[MAX_NUM_SENSORS][3];        // PD normals
  float acc_bias[3];                        // Acceleromater bias
  float acc_scale[3];                       // Accelerometer scale
  float gyr_bias[3];                        // Gyro bias
  float gyr_scale[3];                       // Gyro scale
};

typedef struct {
  uint32_t sweep_time[MAX_NUM_SENSORS];
  uint16_t sweep_len[MAX_NUM_SENSORS];
} lightcaps_sweep_data;

typedef struct {
  int recent_sync_time;
  int activeSweepStartTime;
  int activeLighthouse;
  int activeAcode;
  int lh_start_time[MAX_NUM_LIGHTHOUSES];
  int lh_max_pulse_length[MAX_NUM_LIGHTHOUSES];
  int8_t lh_acode[MAX_NUM_LIGHTHOUSES];
  int current_lh;
} per_sweep_data;

typedef struct {
  double acode_offset;
} global_data;

typedef struct {
  lightcaps_sweep_data sweep;
  per_sweep_data per_sweep;
  global_data global;
} lightcap_data;

// OOTX state machine for each lighthouse
typedef enum {PREAMBLE, LENGTH, PAYLOAD, CHECKSUM} State;
typedef struct {
  State state;                    // Current RX state
  uint8_t data[MAX_PACKET_LEN];   // Data buffer
  uint16_t length;                // Length in bytes
  uint8_t pad;                    // Padding length in bytes : 0 or 1
  uint16_t pos;                   // Bit position
  uint16_t syn;                   // Sync bit counter
  uint32_t crc;                   // CRC32 Checksum
  uint8_t preamble;               // Preamble
  uint32_t lasttime;              // Last sync time
  struct Lighthouse *lighthouse;  // Lighthouse reference...
} OOTX;

// Information about a tracked device
struct Tracker {
  uint16_t type;                            // Tracker type
  struct Driver * driver;                   // Parent driver
  struct libusb_device_handle * udev;       // Udev handle
  char serial[MAX_SERIAL_LENGTH];           // Serial number
  struct Endpoint endpoints[MAX_ENDPOINTS]; // USB endpoints
  struct Calibration cal;                   // Calibration data
  lightcap_data lcd;                        // Lightcap data
  OOTX ootx[MAX_NUM_LIGHTHOUSES];           // OOTX data
  uint8_t charge;                           // Current charge
  uint8_t ischarging:1;                     // Charging?
  uint8_t ison:1;                           // Turned on?
  uint8_t axis[3];                          // Gravitational axis
  uint8_t buttonmask;                       // Buttom mask
  uint32_t timecode;                        // Timecode of last update
};

// Motor information
typedef enum {
  MOTOR_VERTICAL   = 0,
  MOTOR_HORIZONTAL = 1,
  MAX_NUM_MOTORS   = 2
} MotorType;

/* Phase is indeed an offset correction to bearing angles from
   the sync rising edge to hit centroid. Tilt is the clocking of
   the fan beam lens away from its ideal; beam parallel to axis
   of rotation. Curve is an approximate correction for the lens
   axis not being perfectly radial which causes the fan beam
   to curve into a conic instead of being perfectly planar.
   Gibbous is a correction for the laser/mirror/lens alignment
   errors, it is a 2pi periodic correction to bearing angles 
   expressed as a phase and magnitude. You can think of it as
   the 1st Fourier term while phase is the 0th.                */
struct Motor {
  float phase;
  float tilt;
  float gibphase;
  float gibmag;
  float curve;
};

// Lighthouse information
struct Lighthouse {
  uint32_t timestamp;                   // Time of last update (0 = invalud)
  uint8_t id;                           // ID of this lighthouse
  uint16_t fw_version;                  // Firmware version
  char serial[MAX_SERIAL_LENGTH];       // Unique serial number
  struct Motor motors[MAX_NUM_MOTORS];  // Motor calibration data
  float accel[3];                       // acceleration vector
  uint8_t sys_unlock_count;             // Lowest 8 bits of desynchronization
  uint8_t hw_version;                   // Hardware version
  uint8_t mode_current;                 // Current mode (default: 0=A, 1=B, 2=C)
  uint8_t sys_faults;                   // "fault detect flags" (should be 0)
};

// General configuration
struct General {
  int32_t timebase_hz;            // 48,000,000 (checked)
  int32_t timecenter_ticks;       // 200,000    (checked)  (2x = sweep length)
  int32_t pulsedist_max_ticks;    // 500,000    (guessed)
  int32_t pulselength_min_sync;   // 2,200      (guessed)
  int32_t pulse_in_clear_time;    // 35,000     (guessed)
  int32_t pulse_max_for_sweep;    // 1,800      (guessed)
  int32_t pulse_synctime_offset;  // 20,000     (guessed)
  int32_t pulse_synctime_slack;   // 5,000      (guessed)
};

// Callbacks
typedef void (*lig_func)(struct Tracker * tracker, struct Lighthouse * lighthouse,
  uint8_t axis, uint32_t synctime, uint16_t num_sensors,
  uint16_t *sensors,  uint32_t *sweeptimes, uint32_t *angles, uint16_t *lengths);
typedef void (*imu_func)(struct Tracker * tracker, uint32_t timecode,
  int16_t acc[3], int16_t gyr[3], int16_t mag[3]);
typedef void (*but_func)(struct Tracker * tracker, uint32_t timecode,
  uint8_t mask);
typedef void (*tracker_func)(struct Tracker * tracker);
typedef void (*lighthouse_func)(struct Lighthouse * lighthouse);
typedef void (*general_func)(struct General * general);

// Driver context
struct Driver {
  struct libusb_context* usb;
  uint16_t num_trackers;
  struct Tracker **trackers;
  lig_func lig_fn;               // Called when new light data arrives
  imu_func imu_fn;               // Called when new IMU data arrives
  but_func but_fn;               // Called when new button data arrives
  tracker_func tracker_fn;       // Called when tracker cal info is ready
  lighthouse_func lighthouse_fn; // Called when lighthouse cal info is ready
  general_func general_fn;       // Called when general cal info is ready
  struct Lighthouse lighthouses[MAX_NUM_LIGHTHOUSES];
  struct General general;        // General configuration
  uint8_t pushed;                // Have we pushed thie tracker/general config
};

// Initialize the driver
struct Driver * deepdive_init();

// Register a light callback function
void deepdive_install_lig_fn(struct Driver * drv, lig_func fbp);

// Register an IMU callback function
void deepdive_install_imu_fn(struct Driver * drv, imu_func fbp);

// Register an button callback function
void deepdive_install_but_fn(struct Driver * drv, but_func fbp);

// Register an button callback function
void deepdive_install_tracker_fn(struct Driver * drv, tracker_func fbp);

// Register an button callback function
void deepdive_install_lighthouse_fn(struct Driver * drv, lighthouse_func fbp);

// Register an button callback function
void deepdive_install_general_fn(struct Driver * drv, general_func fbp);

// Get the general configuration data
struct General * deepdive_general(struct Driver * drv);

// Get the calibration data for the lighthouse with the given serial number
struct Lighthouse * deepdive_lighthouse(struct Driver * drv, const char* id);

// Get the calibration data for a tracker with the given serial number
struct Tracker * deepdive_tracker(struct Driver * tracker, const char* id);

// Poll the driver for events
int deepdive_poll(struct Driver * drv);

// Close the driver and clean up memory
void deepdive_close(struct Driver * drv);

#endif
