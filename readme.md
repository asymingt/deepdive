# Overview

This is essentially an adaptation of libsurvive for trackers only,
targeted at Ubuntu/Debian systems. Although still written in C, much
of the restructured to be easier to maintain and enhance.

The key changes are the following:

  1. Only linux is supported using libusb (not HIDAPI). I have removed
     FreeBSD, OSX and Windows headers to keep the code easy to read.
 
  2. Only three dependencies -- libjson-c, zlib and libusb-1.0 -- all
     of which are available from the Ubuntu package manager.

  3. A fairly reliable OOTX decode from 1 or 2 base stations. The OOTX
     contains the base station calibration data, which one needs to
     achieve sub-millimeter tracking accuracy.

  4. Calibration data is written to data structure and can be queried
     by serial number through the library API call. A similar approach
     is taken to save and query lighthouse data.

  5. The posers have been removed in favour code simplicity. There are
     many approaches to solving the tracking problem (particle filters,
     Kalman filters, non-linear least squares, etc.) and they tend to
     be quite computationally-intense. I have chosen to deal with these
     outside of the core library.

  6. The only endpoint device supported are trackers. This is because
     this library is targeted at ground truth systems for robotics
     research applications, where headsets aren't used.

  7. I've migrated from straight Makefiles to cmake. This also helps
     a little with resolving dependencies in the build process.

Things I'd like to to:

  1. Neaten up the watchman decode function. It was copied from
     libsurvive and it works, but the code itself is difficult to follow.

  2. Understand the frame in which IMU scale and bias calibration is
     performed. I have a suspicion that the scale and bias is expressed
     in the IMU frame (which is defined by the order of the bytes in
     either the watchman or tracker data streams).

  3. Understand how the light measurements are corrected using light
     data. @nairol has produced a great video explaining what these
     parameters might mean: https://www.youtube.com/watch?v=iVg9_21fcE8

# Copying (MIT licence)

This code was originally forked from libsurvive, an incredible library
developed by the open source community, using hints from those who
built the system itself. There is an active community on Discord, which
I encourage interested people to check out.

I used documentation from: https://github.com/nairol/LighthouseRedox
The code was adapted from: https://github.com/cnlohr/libsurvive
Which was based off: https://github.com/collabora/OSVR-Vive-Libre
  Originally Copyright 2016 Philipp Zabel
  Originally Copyright 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
  Originally Copyright (C) 2013 Fredrik Hultin
  Originally Copyright (C) 2013 Jakob Bornecrantz

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