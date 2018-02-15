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

#include <argtable2.h>

#include <deepdive.h>

// Callback to display light info
void my_light_process(struct Tracker * tracker, struct Lighthouse * lighthouse,
  uint8_t axis, uint32_t synctime, uint16_t num_sensors, uint16_t *sensors,
    uint32_t *sweeptimes, uint32_t *angles, uint16_t *lengths) {
  static float angcnv, lencnv;
  // Print header info
  printf("[%u] # %s LH %s AXIS %c\n", synctime, tracker->serial,
    lighthouse->serial, (axis ? 'Y' : 'X'));
  // Print sensor info
  for (uint16_t i = 0; i < num_sensors; i++) {
    angcnv = (180.0 / 400000.0) * ((float)(angles[i]) - 200000.0);
    lencnv = ((float)lengths[i]) / 48000000.0 * 1000000.0;
    printf(" ->  SEN (%2u) ANG (%f deg) LEN (%f us)\n",
      sensors[i], angcnv, lencnv);
  }
}

// Callback to display imu info
void my_imu_process(struct Tracker * tracker, uint32_t timecode,
  int16_t acc[3], int16_t gyr[3], int16_t mag[3]) {
  static float a[3], g[3];
  a[0] = (float)(acc[0]) * (9.80665/4096.0);
  a[1] = (float)(acc[1]) * (9.80665/4096.0);
  a[2] = (float)(acc[2]) * (9.80665/4096.0);
  g[0] = (float)(gyr[0]) * ((1./32.768)*(3.14159/180.));
  g[1] = (float)(gyr[1]) * ((1./32.768)*(3.14159/180.));
  g[2] = (float)(gyr[2]) * ((1./32.768)*(3.14159/180.));
  printf("[%010u] # %s I - ACC (%7.3f,%7.3f,%7.3f) GYR (%7.3f,%7.3f,%7.3f)\n",
    timecode, tracker->serial, a[0], a[1], a[2], g[0], g[1], g[2]);
}

// Callback to display button info
void my_button_process(struct Tracker * tracker,
  uint32_t mask, uint16_t trigger, int16_t horizontal, int16_t vertical) {
  if (mask & BUTTON_TRIGGER)
    printf("[EVENT] TRIGGER_CLICK\n");
  else if (mask & BUTTON_GRIP)
    printf("[EVENT] GRIP\n");
  else if (mask & BUTTON_MENU)
    printf("[EVENT] MENU\n");
  else if (mask & BUTTON_PAD_CLICK)
    printf("[EVENT] PAD_CLICK - %hd %hd\n", horizontal, vertical);
  else if (mask & BUTTON_PAD_TOUCH)
    printf("[EVENT] PAD_TOUCH - %hd %hd\n", horizontal, vertical);
  else
    printf("[EVENT] TRIGGER - %hu\n", trigger);
}

// Main entry point for application
int main(int argc, char **argv) {
  // Get commandline arguments
  struct arg_lit  *imu     = arg_lit0("i", NULL, "print imu");
  struct arg_lit  *light   = arg_lit0("l", NULL, "print light");
  struct arg_lit  *button  = arg_lit0("b", NULL, "print buttons");
  struct arg_lit  *help    = arg_lit0(NULL, "help", "print this help and exit");
  struct arg_end  *end     = arg_end(20);
  void* argtable[] = {imu, light, button, help, end};
  // Verify we allocated correcty
  const char* progname = "deepdive_tool";
  int nerrors, exitcode = 0;
  if (arg_nullcheck(argtable) != 0) {
    printf("%s: insufficient memory\n", progname);
    exitcode = 1;
    goto exit;
  }
  // Parse the arguments
  nerrors = arg_parse(argc, argv, argtable);
  // Help takes precedence
  if (help->count > 0) {
    printf("Usage: %s", progname);
    arg_print_syntax(stdout, argtable, "\n");
    printf("This program extracts and prints data from a vive system.\n");
    arg_print_glossary(stdout, argtable,"  %-25s %s\n");
    exitcode = 0;
    goto exit;
  }
  /* If the parser returned any errors then display them and exit */
  if (nerrors > 0) {
    /* Display the error details contained in the arg_end struct.*/
    arg_print_errors(stdout,end,progname);
    printf("Try '%s --help' for more information.\n", progname);
    exitcode = 2;
    goto exit;
  }
  // Initialize the driver
  struct Driver * drv = deepdive_init(0);
  if (!drv) {
    printf("%s: could not initialize driver\n", progname);
    exitcode = 3;
    goto exit;
  }
  // Install callbacks
  if (imu->count > 0)
    deepdive_install_imu_fn(drv, my_imu_process);
  if (light->count > 0)
    deepdive_install_light_fn(drv, my_light_process);
  if (button->count > 0)
    deepdive_install_button_fn(drv, my_button_process);
  // Keep going until ctrl+c
  while(deepdive_poll(drv) == 0) {}
    return 0;
  // Exit cleanly
  exitcode = 0;
exit:
  arg_freetable(argtable,sizeof(argtable)/sizeof(argtable[0]));
  return exitcode;
}

