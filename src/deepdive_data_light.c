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

#include "deepdive_data_light.h"

#include <zlib.h>

// OOTX constants
#define MAX_PACKET_LEN  64
#define PREAMBLE_LENGTH 17
#define SYNC_VALUE      0x1
#define STEP_SIZE       400000
#define STEP_VAR        250
#define SYNC_TIMEOUT    10000000

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
  uint8_t id;                     // Lighthouse id
} OOTX;

// Avoids compiler reording with -O2
union custom_float {
  uint32_t i;
  float f;
};

// Converts a 16bit float to a 32 bit float
static float convert_float(uint8_t* data) {
  uint16_t x = *(uint16_t*)data;
  union custom_float fnum;
  fnum.f = 0;
  // handle sign
  fnum.i = (x & 0x8000)<<16;
  if ((x & 0x7FFF) == 0) return fnum.f; //signed zero
  if ((x & 0x7c00) == 0) {
    // denormalized
    x = (x&0x3ff)<<1; // only mantissa, advance intrinsic bit forward
    uint8_t e = 0;
    // shift until intrinsic bit of mantissa overflows into exponent
    // increment exponent each time
    while ((x&0x0400) == 0) {
      x<<=1;
      e++;
    }
    fnum.i |= ((uint32_t)(112-e))<<23;    // bias exponent to 127
    fnum.i |= ((uint32_t)(x&0x3ff))<<13;  // insert mantissa
    return fnum.f;
  }
  if((x&0x7c00) == 0x7c00) {
    // for infinity, fraction is 0; for NaN, fraction is anything non zero
    // we shift because the mantissa of a NaN can have meaning
    fnum.i |= 0x7f800000 | ((uint32_t)(x & 0x3ff))<<13;
    return fnum.f;
  }
  fnum.i |= ((((uint32_t)(x & 0x7fff)) + 0x1c000u) << 13);
  return fnum.f;
}

// Convert the packet
static void decode_packet(struct Lighthouse * lh, uint8_t *data, uint32_t tc) {
  sprintf(lh->serial, "%u", *(uint32_t*)(data + 0x02));
  lh->fw_version = *(uint16_t*)(data + 0x00);
  lh->motors[0].phase = convert_float(data + 0x06);
  lh->motors[1].phase = convert_float(data + 0x08);
  lh->motors[0].tilt = convert_float(data + 0x0a);
  lh->motors[1].tilt = convert_float(data + 0x0c);
  lh->sys_unlock_count = *(uint8_t*)(data + 0x0e);
  lh->hw_version = *(uint8_t*)(data + 0x0f);
  lh->motors[0].curve = convert_float(data + 0x10);
  lh->motors[1].curve = convert_float(data + 0x12);
  lh->accel[0] = *(int8_t*)(data + 0x14);
  lh->accel[1] = *(int8_t*)(data + 0x15);
  lh->accel[2] = *(int8_t*)(data + 0x16);
  lh->motors[0].gibphase = convert_float(data + 0x17);
  lh->motors[1].gibphase = convert_float(data + 0x19);
  lh->motors[0].gibmag = convert_float(data + 0x1b);
  lh->motors[1].gibmag = convert_float(data + 0x1d);
  lh->mode_current = *(int8_t*)(data + 0x1f);
  lh->sys_faults = *(int8_t*)(data + 0x20);
  lh->timestamp = tc;
  printf("[%10u] RX lighthouse config for #%s\n", tc, lh->serial);
  /*
  printf("Lighthouse FW: %u\n", lh->fw_version);
  printf("Lighthouse MODE: %c\n", (lh->mode_current == 1 ? 'b' : 'c'));
  printf("Lighthouse phase motor 0: %f\n", lh->motors[0].phase);
  printf("Lighthouse tilt motor 0: %f\n", lh->motors[0].tilt);
  printf("Lighthouse phase motor 1: %f\n", lh->motors[1].phase);
  printf("Lighthouse tilt motor 1: %f\n", lh->motors[1].tilt);
  */
}

// Swap endianness of 16 bit unsigned integer
static uint16_t swaps(uint16_t val) {
    return    ((val << 8) & 0xff00) 
            | ((val >> 8) & 0x00ff);
}

// Swap endianness of 32 bit unsigned integer
static uint32_t swapl(uint32_t val) {
  return      ((val >> 24) & 0x000000ff)
            | ((val << 8)  & 0x00ff0000)
            | ((val >> 8)  & 0x0000ff00)
            | ((val << 24) & 0xff000000);
}

// Process a single bit of the OOTX data
static void ootx_feed(struct Driver *drv,OOTX* ctx, uint8_t bit, uint32_t tc) {
  // Always check for preamble and reset if needed
  if (bit) {
    if (ctx->preamble >= PREAMBLE_LENGTH) {
      //printf("Preamble found\n");
      ctx->state = LENGTH;
      ctx->length = 0;
      ctx->pos = 0;
      ctx->syn = 0;
      ctx->preamble = 0;
      return;
    }
    ctx->preamble = 0;
  } else {
    ctx->preamble++;
  }
  
  // State machine
  switch (ctx->state) {
  case PREAMBLE:
    return;
  case LENGTH:
    if (ctx->syn == 16) {
      ctx->length = swaps(ctx->length);   // LE to BE
      //printf("LEN: %u for LH %u\n", ctx->length, ctx->id);
      ctx->pad = (ctx->length % 2);       // Force even num bytes
      //printf("PAD: %u for LH %u\n", ctx->pad , ctx->id);
      ctx->state = PREAMBLE;
      if (ctx->length + ctx->pad <= MAX_PACKET_LEN) {
        //printf("[PRE] -> [PAY]\n");
        ctx->state = PAYLOAD;
        ctx->syn = ctx->pos = 0;
        memset(ctx->data, 0x0, MAX_PACKET_LEN);
      }
      return;
    }
    ctx->length |= (((uint16_t)bit) << (15 - ctx->syn++));
    return;
  case PAYLOAD:
    // Decrement the pointer every 8 bits to find the byte offset
    if (ctx->syn == 8 || ctx->syn == 16) {
      // Increment the byte offset
      ctx->pos++;
      // If we can't decrement pointer then we have received all the data
      if (ctx->pos == ctx->length + ctx->pad) {
        //printf("[PAY] -> [CRC]\n");
        ctx->state = CHECKSUM;
        ctx->syn = ctx->pos = 0;
        ctx->crc = 0;
        return;
      }
    }
    // The 17th bit is a sync bit, and should be swallowed
    if (ctx->syn == 16) {
      ctx->syn = 0;
      return;
    }
    // Append data to the current byte in the sequence
    ctx->data[ctx->pos] |= (bit << (7 - ctx->syn++ % 8));
    return;
  case CHECKSUM:
    // Decrement the pointer every 8 bits to find the byte offset
    if (ctx->syn == 8 || ctx->syn == 16) {
      ctx->pos++;
      if (ctx->pos == 4) {
        // Calculate the CRC
        uint32_t crc = crc32( 0L, 0 /*Z_NULL*/, 0 );
        crc = crc32(crc, ctx->data, ctx->length);
        // Print some debug info
        //printf("[CRC] -> [PRE]\n");
        //printf("[CRC] RX = %08x\n", swapl(ctx->crc));
        //printf("[CRC] CA = %08x\n", crc);
        if (crc == swapl(ctx->crc))
          decode_packet(&drv->lighthouses[ctx->id], ctx->data, tc);
        // Return to state
        ctx->state = PREAMBLE;
        ctx->pos = ctx->syn = 0;
        ctx->preamble = 0;
        ctx->length = 0;
        return;
      }
    }
    // The 17th bit is a sync bit, and should be swallowed
    if (ctx->syn == 16) {
      ctx->syn = 0;
      return;
    }
    ctx->crc |= (((uint32_t)bit) << (31 - (ctx->pos * 8 + ctx->syn++ % 8)));
    return;
  }
}

// Get the acode from the sync pulse length
static uint8_t get_acode_from_sync_pulse(uint16_t len) {
  return (uint8_t)((len - 2750) / 500);
}

// Get the lighthouse from the time code
static uint8_t timecodes_match(uint32_t ref, uint32_t sample) {
  static uint32_t diff;
  if (ref == 0)
    return 1;
  if (ref > sample)
    return 0;
  diff = (sample - ref) % STEP_SIZE;
  // printf("%u\n", diff);
  return ((diff < STEP_VAR || diff > STEP_SIZE - STEP_VAR) ? 1 : 0);
}

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

// Process light data
void deepdive_data_light(struct Tracker * tracker,
  uint32_t timecode, uint16_t sensor, uint16_t length) {
  // Statically allocated so we don't need to attach to the Tracker context
  static uint32_t last_tc = 0;
  static uint8_t  last_lh = MAX_NUM_LIGHTHOUSES;
  static uint8_t  last_ax = MAX_NUM_MOTORS;
  static OOTX ootx[MAX_NUM_LIGHTHOUSES];
  ootx[0].id = 0;
  ootx[1].id = 1;
  // Reject sensor or pulse lengths that are too high
  if (sensor > MAX_NUM_SENSORS || length > 6750)
    return;
  // Pulse durations greater than 2750 are sync packets, since the
  // maximum pulse duration of a sweep is well below this value.
  if (length > 2750) {
    // Get the acode from the sync packet
    uint8_t acode = get_acode_from_sync_pulse(length);
    uint8_t le = (acode & 0x4) ? 1 : 0; // Laser enabled
    uint8_t da = (acode & 0x2) ? 1 : 0; // Data bit
    uint8_t ax = (acode & 0x1) ? 1 : 0; // Laser axis
    uint8_t lh = MAX_NUM_LIGHTHOUSES;   // Unknown
    // Now we don't know which lighthouse this acode actually came from,
    // because both lighthouses continually transmit all 8 possible codes.
    // So, we're going to have to resolve this using the timecode. I am
    // assuming there is a relativey constant stride between pulses. In this
    // case any two timecodes (m,n) from a single lighthouse will satisfy:
    //     (n - m) % 4000000 < 2
    // This approach is not perfect, but it seems to work in practice.
    for (uint8_t i = 0; i < MAX_NUM_LIGHTHOUSES; i++) {
      if (timecode - ootx[i].lasttime > SYNC_TIMEOUT)
        ootx[i].lasttime = timecode;
      if (timecodes_match(ootx[i].lasttime, timecode)) {
        ootx_feed(tracker->driver, &ootx[i], da, timecode);
        ootx[i].lasttime = timecode;
        lh = i;
        break;
      }
    }
    // If the laser was not enabled on this spin, then we should not update
    // the per-sweep information, since we don't expect any pulses from it!
    if (!le) return;
    // If we managed to determine the lighthouse from which the sync was
    // produced, when we can adjust the "last" values, which persist.
    if (lh < MAX_NUM_LIGHTHOUSES) {
      last_tc = timecode;
      last_ax = ax;
      last_lh = lh;
      return;
    }
    // If we get here then it means that our laser was enabled but we don't
    // know the lighthouse. So, we should discard the information.
    last_lh = MAX_NUM_LIGHTHOUSES;
    return;
  // Everything else is a sensor measurement. We should only process
  // the measurement if we know the lh and axis from the last sweep.
  } else if (last_lh < MAX_NUM_LIGHTHOUSES) {
    if (tracker->driver->lig_fn) {
      tracker->driver->lig_fn(tracker, last_tc, last_lh,
        last_ax, sensor, timecode - last_tc, length);
    }
  }
}