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

// Interface implementations
#include "deepdive_usb.h"

// Controller implementations
#include "deepdive_dev_tracker.h"
#include "deepdive_dev_watchman.h"

#include <json/json.h>

#include <stdio.h>
#include <errno.h>
#include <zlib.h>

// Special codes
static uint8_t magic_code_power_en_[5] = {0x04};

// Interrupt handler
static void interrupt_handler(struct libusb_transfer* t) {
  struct Endpoint *ep = t->user_data;
  if (t->status != LIBUSB_TRANSFER_COMPLETED ) {
    printf("Transfer problem\n");
    return;
  }
  switch (ep->type) {
   case TRACKER_IMU:
    deepdive_dev_tracker_imu(ep->tracker, ep->buffer, t->actual_length);
    break;
   case TRACKER_LIGHT:
    deepdive_dev_tracker_light(ep->tracker, ep->buffer, t->actual_length);
    break;
   case TRACKER_BUTTONS:
    deepdive_dev_tracker_button(ep->tracker, ep->buffer, t->actual_length);
    break;
   case WATCHMAN:
    deepdive_dev_watchman(ep->tracker, ep->buffer, t->actual_length);
    break;
  }
  if (libusb_submit_transfer(t))
    printf( "Error resubmitting transfer\n");
}

static inline int update_feature_report(libusb_device_handle* dev,
  uint16_t interface, uint8_t * data, int datalen) {
  return libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_CLASS
    | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT, 0x09, 0x300 | data[0],
      interface, data, datalen, 1000 );
}


static inline int getupdate_feature_report(libusb_device_handle* dev,
  uint16_t interface, uint8_t * data, int datalen ) {
  return libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_CLASS
    | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_IN, 0x01, 0x300 | data[0],
    interface, data, datalen, 1000 );
}

static inline int hid_get_feature_report_timeout(libusb_device_handle* dev,
  uint16_t iface, unsigned char *buf, size_t len ) {
  int ret;
  for (uint8_t i = 0; i < 50; i++) {
    ret = getupdate_feature_report(dev, iface, buf, len);
    if (ret != -9 && (ret != -1 || errno != EPIPE))
      return ret;
    usleep(1000);
  }
  return -1;
}

static int decompress(const char * input, int ilen, char * output, int olen) {
  z_stream zs;
  memset(&zs, 0, sizeof( zs ));
  inflateInit(&zs);
  zs.avail_in = ilen;
  zs.next_in = (z_const Bytef *)input;
  zs.avail_out = olen;
  zs.next_out = output;
  if(inflate( &zs, Z_FINISH) != Z_STREAM_END) {
    printf("Could not inflate.");
    return -1;
  }
  int len = zs.total_out;
  inflateEnd(&zs);
  return len;
}

// Read an array from a json array data structure
static int json_read_arr_dbl(json_object * jobj, float *data, size_t len) {
  // Check that this is an array
  enum json_type type = json_object_get_type(jobj);
  if (type != json_type_array)
    return 0;
  // Read the min(desired,actual) length
  int maxlen = json_object_array_length(jobj);
  if (maxlen < len)
    len = maxlen;
  // Now read the elements
  json_object * jval;
  int numread = 0;
  for (size_t i = 0; i < len; i++) {
    jval = json_object_array_get_idx(jobj, i);
    data[numread++] =  json_object_get_double(jval);
  }
  return numread;
}

// Read an array from a json array data structure
static int json_read_arr_int(json_object * jobj, uint8_t *data, size_t len) {
  // Check that this is an array
  enum json_type type = json_object_get_type(jobj);
  if (type != json_type_array)
    return 0;
  // Read the min(desired,actual) length
  int maxlen = json_object_array_length(jobj);
  if (maxlen < len)
    len = maxlen;
  // Now read the elements
  json_object * jval;
  int numread = 0;
  for (size_t  i = 0; i < len; i++) {
    jval = json_object_array_get_idx(jobj, i);
    data[numread++] = json_object_get_int(jval);
  }
  return numread;
}

// Parse the jscond evice configuation
static int json_parse(struct Tracker * tracker, const char* data) {
  json_object *jobj = json_tokener_parse(data);
  json_object *jtmp;
  // IMU calibration parameters
  if (json_object_object_get_ex(jobj, "device_serial_number", &jtmp))
    strcpy(tracker->serial, json_object_get_string(jtmp));
  if (json_object_object_get_ex(jobj, "acc_bias", &jtmp))
    if (!json_read_arr_dbl(jtmp, tracker->cal.acc_bias, 3))
      printf("Could not read the JSON field: acc_bias\n");
  if (json_object_object_get_ex(jobj, "acc_scale", &jtmp))
    if (!json_read_arr_dbl(jtmp, tracker->cal.acc_scale, 3))
      printf("Could not read the JSON field: acc_scale\n");
  if (json_object_object_get_ex(jobj, "gyro_bias", &jtmp))
    if (!json_read_arr_dbl(jtmp, tracker->cal.gyr_bias, 3))
      printf("Could not read the JSON field: gyr_bias\n");
  if (json_object_object_get_ex(jobj, "gyro_scale", &jtmp))
    if (!json_read_arr_dbl(jtmp, tracker->cal.gyr_scale, 3))
      printf("Could not read the JSON field: gyr_scale\n");
  if (json_object_object_get_ex(jobj, "trackref_from_imu", &jtmp))
    if (!json_read_arr_dbl(jtmp, tracker->cal.imu_transform, 7))
      printf("Could not read the JSON field: trackref_from_imu\n");
  if (json_object_object_get_ex(jobj, "trackref_from_head", &jtmp))
    if (!json_read_arr_dbl(jtmp, tracker->cal.head_transform, 7))
      printf("Could not read the JSON field: trackref_from_head\n");
  // Photodiode calibration parameters
  json_object *jlhc;
  if (json_object_object_get_ex(jobj, "lighthouse_config", &jlhc)) {
    if (json_object_object_get_ex(jlhc, "channelMap", &jtmp)) {
      tracker->cal.num_channels = json_read_arr_int(
        jtmp, tracker->cal.channels, MAX_NUM_SENSORS);
      if (tracker->cal.num_channels > 0
       && tracker->cal.num_channels < MAX_NUM_SENSORS) {
        json_object *jnrm;
        if (json_object_object_get_ex(jlhc, "modelNormals", &jnrm)) {
          for (int i = 0; i < tracker->cal.num_channels; i++) {
            jtmp = json_object_array_get_idx(jnrm, i);
            if (!json_read_arr_dbl(jtmp, tracker->cal.normals[i], 3))
              printf("Could not read the normal for channel %u\n", i);
          }
        }
        json_object *jpts;
        if (json_object_object_get_ex(jlhc, "modelPoints", &jpts)) {
          for (int i = 0; i < tracker->cal.num_channels; i++) {
            jtmp = json_object_array_get_idx(jpts, i);
            if (!json_read_arr_dbl(jtmp, tracker->cal.positions[i], 3))
              printf("Could not read the position for channel %u\n", i);
          }
        }
      } else {
        printf("Could not read the JSON: lighthouse_config::channelMap\n");
      }
    }
  }
  // Mark as valid!
  tracker->cal.timestamp = 1;
  printf("Read calibration data for tracker %s\n", tracker->serial);
}

// Read the tracker configuration (sensor extrinsics and imu bias/scale)
static int get_config(struct Tracker * tracker, int send_extra_magic) {
  int ret, count = 0, size = 0;
  uint8_t cfgbuff[64];
  uint8_t compressed_data[8192];
  uint8_t uncompressed_data[65536];
  // Send a magic code to iniitalize the config download process
  if (send_extra_magic) {
    uint8_t cfgbuffwide[65];
    memset(cfgbuffwide, 0, sizeof(cfgbuff));
    cfgbuffwide[0] = 0x01;
    ret = hid_get_feature_report_timeout(
      tracker->udev, 0, cfgbuffwide,sizeof(cfgbuffwide) );
    usleep(1000);
    uint8_t cfgbuff_send[64] = { 0xff, 0x83 };
    for (int k = 0; k < 10; k++ ) {
      update_feature_report(tracker->udev, 0, cfgbuff_send, 64 );
      usleep(1000);
    }
    cfgbuffwide[0] = 0xff;
    ret = hid_get_feature_report_timeout(
      tracker->udev, 0, cfgbuffwide, sizeof(cfgbuffwide));
    usleep(1000);
  }
  // Send Report 16 to prepare the device for reading config info
  memset(cfgbuff, 0, sizeof(cfgbuff));
  cfgbuff[0] = 0x10;
  if((ret = hid_get_feature_report_timeout(
    tracker->udev, 0, cfgbuff, sizeof(cfgbuff) ) ) < 0 ) {
    printf( "Could not get survive config data for device");
    return -1;
  }
  // Now do a bunch of Report 17 until there are no bytes left
  cfgbuff[1] = 0xaa;
  cfgbuff[0] = 0x11;
  do {
    if((ret = hid_get_feature_report_timeout(
        tracker->udev, 0, cfgbuff, sizeof(cfgbuff))) < 0 ) {
      printf("Could not read config data on device (count: %d)\n", count );
      return -2;
    }
    size = cfgbuff[1];
    if (!size) break;
    if( size > 62 ) {
      printf("Too much data (%d) on packet (count: %d)", size, count);
      return -3;
    }
    if (count + size >= sizeof(compressed_data)) {
      printf("Configuration length too long (count: %d)", count);
      return -4;
    }
    memcpy(&compressed_data[count], cfgbuff + 2, size);
    count += size;
  } while( 1 );

  if (count == 0) {
    printf( "Empty configuration");
    return -5;
  }
  // Decompress the data
  int len = decompress(compressed_data, count,
    uncompressed_data, sizeof(uncompressed_data));
  if (len <= 0) {
    printf( "Error: data for config descriptor is bad. (%d)", len);
    return -5;
  }
  /*
  char fstname[128];
  sprintf(fstname, "%s.json.gz", tracker->serial);
  FILE *f = fopen( fstname, "wb" );
  fwrite(uncompressed_data, len, 1, f);
  fclose(f);
  */
  // Parse the JSON data structure
  json_parse(tracker, uncompressed_data);
  return 0;
}

// Enumerate all USBs on the bus and return the number of devices found
int deepdive_usb_init(struct Driver * drv) {
  // Initialize libusb
  int ret = libusb_init(&drv->usb);
  if (ret)
    return 0;

  // Get a list of devices
  libusb_device** devs;
  ret = libusb_get_device_list(drv->usb, &devs);
  if (ret < 0)
    return 0;

  // Iterate over the devices looking for vive products
  struct libusb_device_descriptor desc;
  struct libusb_device * dev;
  for (int did = 0; dev = devs[did]; did++) {
    // Get the device descriptor
    ret = libusb_get_device_descriptor(dev, &desc);
    if (ret < 0 || dev == 0 || desc.idVendor != USB_VEND_HTC)
      continue;

    // Get a config descriptor
    struct libusb_config_descriptor *conf;
    ret = libusb_get_config_descriptor(dev, 0, &conf);
    if (ret)
      continue;

    // Allocate the tracker memory
    struct Tracker *tracker = malloc(sizeof(struct Tracker));
    if (!tracker)
      continue;
    // Make sure the memory is zeroed
    memset(tracker, 0, sizeof(struct Tracker));
    tracker->driver = drv;

    // Null the lighthouse pointer
    for (size_t i = 0; i < MAX_NUM_LIGHTHOUSES; i++)
      tracker->ootx[i].lighthouse = NULL;

    // Try and open the device
    ret = libusb_open(dev, &tracker->udev);
    if (ret || !tracker->udev)
      goto fail;

    // Set to auto-detatch
    libusb_set_auto_detach_kernel_driver(tracker->udev, 1);
    for (int j = 0; j < conf->bNumInterfaces; j++)
      if (libusb_claim_interface(tracker->udev, j))
        goto fail;

    // Get the serial number from the opened device handle
    ret = libusb_get_string_descriptor_ascii(tracker->udev,
      desc.iSerialNumber, tracker->serial, MAX_SERIAL_LENGTH);
    if (ret < 0)
      goto fail;

    // The tracker type is simply the USB product ID
    tracker->type = desc.idProduct;

    // What we do depends on the product
    switch (tracker->type) {
     ///////////////////////////////
     // USB TRACKER OR CONTROLLER //
     ///////////////////////////////
     default:
      continue;
     case USB_PROD_CONTROLLER:
     case USB_PROD_TRACKER:
      // Endpoint for IMU
      tracker->endpoints[0].tracker = tracker;
      tracker->endpoints[0].type = TRACKER_IMU;
      tracker->endpoints[0].tx = libusb_alloc_transfer(0);
      if (!tracker->endpoints[0].tx)
        goto fail;
      libusb_fill_interrupt_transfer(tracker->endpoints[0].tx, tracker->udev,
        USB_ENDPOINT_GENERAL, tracker->endpoints[0].buffer,
        USB_INT_BUFF_LENGTH, interrupt_handler, &tracker->endpoints[0], 0);
      ret = libusb_submit_transfer(tracker->endpoints[0].tx);
      if(ret)
        goto fail;
      // Endpoint for light
      tracker->endpoints[1].tracker = tracker;
      tracker->endpoints[1].type = TRACKER_LIGHT;
      tracker->endpoints[1].tx = libusb_alloc_transfer(0);
      if (!tracker->endpoints[1].tx)
        goto fail;
      libusb_fill_interrupt_transfer(tracker->endpoints[1].tx, tracker->udev,
        USB_ENDPOINT_LIGHT, tracker->endpoints[1].buffer,
        USB_INT_BUFF_LENGTH, interrupt_handler, &tracker->endpoints[1], 0);
      ret = libusb_submit_transfer(tracker->endpoints[1].tx);
      if(ret)
        goto fail;
      // Endpoint for buttons
      tracker->endpoints[2].tracker = tracker;
      tracker->endpoints[2].type = TRACKER_BUTTONS;
      tracker->endpoints[2].tx = libusb_alloc_transfer(0);
      if (!tracker->endpoints[2].tx)
        goto fail;
      libusb_fill_interrupt_transfer(tracker->endpoints[2].tx, tracker->udev,
        USB_ENDPOINT_BUTTONS, tracker->endpoints[2].buffer,
        USB_INT_BUFF_LENGTH, interrupt_handler, &tracker->endpoints[2], 0);
      ret = libusb_submit_transfer(tracker->endpoints[2].tx);
      if(ret)
        goto fail;
      // Send a magic code to power on the tracker
      if (update_feature_report(tracker->udev, 0, magic_code_power_en_,
        sizeof(magic_code_power_en_)) != sizeof(magic_code_power_en_))
          printf("Power on failed\n");
      else
        printf("Power on success\n");
      // Get the configuration for this device
      ret = get_config(tracker, 0);
      if (ret < 0) {
        printf("Calibration cannot be pulled. Ignoring.\n");
        goto fail;
      }
      printf("Found tracker %s\n", tracker->serial);
      break;
     ///////////////////////
     // WIRELESS WATCHMAN //
     ///////////////////////
     case USB_PROD_WATCHMAN:
      // Set up the interrupts
      tracker->endpoints[0].tracker = tracker;
      tracker->endpoints[0].type = WATCHMAN;
      tracker->endpoints[0].tx = libusb_alloc_transfer(0);
      if (!tracker->endpoints[0].tx)
        goto fail;
      libusb_fill_interrupt_transfer(tracker->endpoints[0].tx, tracker->udev,
        USB_ENDPOINT_GENERAL, tracker->endpoints[0].buffer,
        USB_INT_BUFF_LENGTH, interrupt_handler, &tracker->endpoints[0], 0);
      ret = libusb_submit_transfer(tracker->endpoints[0].tx);
      if (ret)
        goto fail;
      // Get the configuration for this device
      ret = get_config(tracker, 1);
      if (ret < 0) {
        printf("Calibration cannot be pulled. Ignoring.\n");
        goto fail;
      }
      printf("Found watchman %s\n", tracker->serial);
      break;
    }
    // Add the tracker to the dynamic list of trackers
    drv->trackers[drv->num_trackers++] = tracker;
    continue;

    // Catch-all to prevent memory leaks
fail:
     free(tracker);
  }

  // Free the device list
  libusb_free_device_list(devs, 1);

  // Success
  return drv->num_trackers;
}