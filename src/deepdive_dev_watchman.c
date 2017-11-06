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

#include "deepdive_dev_watchman.h"
#include "deepdive_data_imu.h"
#include "deepdive_data_light.h"
#include "deepdive_data_button.h"

// Pop a value off the array (shifts the pointer to the next element)
#define POP1  (*(buf++))
#define POP2  (*(((uint16_t*)((buf+=2)-2))))
#define POP4  (*(((uint32_t*)((buf+=4)-4))))

// Stores a single light capture event
typedef struct {
  uint8_t sensor_id;
  uint16_t length;
  uint32_t timestamp;
} LightcapElement;

// Process watchman data. This is necessarily more complex, as the the
// protocol needs to cram both IMU and light data down a Nordic radio
// with a very limited bandwidth. Refer to the following video:
// o https://www.youtube.com/watch?v=oHJkpNakswM
static void watchman_decode(struct Tracker * tracker, uint8_t *buf) {
  uint8_t time1 = POP1;
  uint8_t qty = POP1;
  uint8_t time2 = POP1;
  uint8_t type = POP1;
  uint8_t buttonmask;

  qty-=2;
  int propset = 0;
  int doimu = 0;

  if ((type & 0xf0) == 0xf0) {
    propset |= 4;
    type &= ~0x10;

    if (type & 0x01) {
      qty-=1;
      uint8_t buttonmask = POP1;
      if (buttonmask != tracker->buttonmask)
        deepdive_data_button(tracker, (time1<<24)|(time2<<16), buttonmask);
      type &= ~0x01;
    }
    if (type & 0x04) {
      qty-=1;
      tracker->axis[0] = ( POP1 ) * 128; 
      type &= ~0x04;
    }
    if (type & 0x02) {
      qty-=4;
      tracker->axis[1] = POP2;
      tracker->axis[2] = POP2;
      type &= ~0x02;
    }

    //XXX TODO: Is this correct?  It looks SO WACKY
    type &= 0x7f;
    if (type == 0x68) doimu = 1;
    type &= 0x0f;
    if (type == 0x00 && qty) {
      type = POP1;
      qty--;
    }
  }

  if (type == 0xe1) {
    propset |= 1;
    tracker->ischarging = buf[0]>>7;
    tracker->charge = POP1 & 0x7f; 
    qty--;
    tracker->ison = 1; 
    if (qty) {
      qty--;
      type = POP1; //IMU usually follows.
    }
  }

  // Hmm, this looks kind of yucky... we can get e8's that are accelgyro's but, cleared by first propset.
  if (((type & 0xe8) == 0xe8) || doimu) {
    propset |= 2;
    // Timecode is wrapping milliseconds
    uint32_t timecode = (time1<<24)|(time2<<16)|buf[0];
    // Accelerometer (moved from imu-frame to tracker-frame)
    int16_t acc[3];
    acc[0] =  *((int16_t*)(buf+1));
    acc[1] = -*((int16_t*)(buf+5));
    acc[2] = -*((int16_t*)(buf+3));
    // Gyro (moved from imu-frame to tracker-frame)
    int16_t gyr[3];
    gyr[0] = -*((int16_t*)(buf+7));
    gyr[1] =  *((int16_t*)(buf+11));
    gyr[2] =  *((int16_t*)(buf+9));
    // Process the data
    /*
    printf("[%u] ", timecode);
    for (size_t i = 1; i < 12; i+=2)
      printf("%06d\t", *((int16_t*)(buf+i)));
    printf(" :: ");
    for (size_t i = 17; i < 24; i+=2)
      printf("%06u\t", *((uint16_t*)(buf+i)));
    printf("\n");
    */

    // Push the IMU event
    deepdive_data_imu(tracker, timecode, acc, gyr, NULL);

    int16_t * k = (int16_t *)buf+1;
    buf += 13; qty -= 13;
    type &= ~0xe8;
    if (qty) {
      qty--;
      type = POP1;
    }
  }


  if( qty ) {
    qty++;
    buf--;
    *buf = type; //Put 'type' back on stack.
    uint8_t * mptr = buf + qty-3-1; //-3 for timecode, -1 to 

    uint32_t mytime = (mptr[3] << 16)|(mptr[2] << 8)|(mptr[1] << 0);

    uint32_t times[20];
    const int nrtime = sizeof(times)/sizeof(uint32_t);
    int timecount = 0;
    int leds;
    int fault = 0;

    ///Handle uint32_tifying (making sure we keep it incrementing)
    uint32_t llt = tracker->timecode;
    uint32_t imumsb = time1<<24;
    mytime |= imumsb;

    //Compare mytime to llt

    int diff = mytime - llt;
    if( diff < -0x1000000 )
      mytime += 0x1000000;
    else if( diff > 0x100000 )
      mytime -= 0x1000000;

    tracker->timecode = mytime;

    times[timecount++] = mytime;
    //First, pull off the times, starting with the current time, then all the delta times going backwards.
    {
      while( mptr - buf > (timecount>>1) )
      {
        uint32_t arcane_value = 0;
        //ArcanePop (Pop off values from the back, forward, checking if the MSB is set)
        do {
          uint8_t ap = *(mptr--);
          arcane_value |= (ap&0x7f);
          if( ap & 0x80 )  break;
          arcane_value <<= 7;
        } while(1);
        times[timecount++] = (mytime -= arcane_value);
      }

      leds = timecount>>1;
      //Check that the # of sensors at the beginning match the # of parameters we would expect.
      if( timecount & 1 ) { fault = 1; goto end; }        //Inordinal LED count
      if( leds != mptr - buf + 1 ) { fault = 2; goto end; }  //LED Count does not line up with parameters
    }


    LightcapElement les[10];
    int lese = 0; //les's end


    //Second, go through all LEDs and extract the lightevent from them. 
    {
      uint8_t *marked;
      marked = alloca(nrtime);
      memset( marked, 0, nrtime );
      int i, parpl = 0;
      timecount--;
      int timepl = 0;

      //This works, but usually returns the values in reverse end-time order.
      for (i = 0; i < leds; i++) {
        int led = buf[i];
        int adv = led & 0x07;
        led >>= 3;

        while (marked[timepl])
          timepl++;

        if (timepl > timecount) {
          fault = 3;
          goto end;
        }
        uint32_t endtime = times[timepl++];

        int end = timepl + adv;
        if( end > timecount ) { fault = 4; goto end; } //end referencing off list
        if( marked[end] > 0 ) { fault = 5; goto end; } //Already marked trying to be used.
        uint32_t starttime = times[end];
        marked[end] = 1;

        //Insert all lighting things into a sorted list.  This list will be
        //reverse sorted, but that is to minimize operations.  To read it
        //in sorted order simply read it back backwards.
        //Use insertion sort, since we should most of the time, be in order.

        // Check for valid pulse length
        if ((uint32_t)(endtime - starttime) > 65535) {
          fault = 6;
          goto end;
        }

        LightcapElement * le = &les[lese++];
        le->sensor_id = led;
        le->length = endtime - starttime;
        le->timestamp = starttime;

        int swap = lese-2;
        while( swap >= 0 && les[swap].timestamp < les[swap+1].timestamp) {
          LightcapElement l;
          memcpy( &l, &les[swap], sizeof( l ) );
          memcpy( &les[swap], &les[swap+1], sizeof( l ) );
          memcpy( &les[swap+1], &l, sizeof( l ) );
          swap--;
        }
      }
    }

    // Push all of the light events
    for (int i = lese-1; i >= 0; i--) {
      deepdive_data_light(tracker,
        les[i].timestamp, les[i].sensor_id, les[i].length);
    }

    return;
end:
    printf("Light decoding fault: %d", fault);
  }
}

// There are three different watchman packet IDs
void deepdive_dev_watchman(struct Tracker * tracker, uint8_t *buf, int32_t len) {
  uint8_t id = POP1;
  switch (id) {
  case 35:
    watchman_decode(tracker, buf);
    break;
  case 36:
    watchman_decode(tracker, buf);
    watchman_decode(tracker, buf + 29);
    break;
  case 38:
    tracker->ison = 0;
    break;
  default:
    printf("Unknown watchman code\n");
  }
}
