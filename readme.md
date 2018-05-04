# Overview

This is an unofficial driver for the HTC Vive tracking system, split into two major components -- a C-based low level driver for connecting to and extracting light and IMU measurements from trackers, and a ROS/C++ high-level driver for working with the raw measurements. Note that this code is not production ready or even working fully, only supports HTC trackers -- not HMDs or controllers -- and has only been tested on on Ubuntu.

# Preparing your system

First, make sure you add the udev rules, otherwise you won't have read access to the HID devices created when you plug in a tracker.

    # HTC Vive HID Sensor naming and permissioning
    KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0bb4", ATTRS{idProduct}=="2c87", TAG+="uaccess"
    KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2101", TAG+="uaccess"
    KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2000", TAG+="uaccess"
    KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="1043", TAG+="uaccess"
    KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2050", TAG+="uaccess"
    KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2011", TAG+="uaccess"
    KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2012", TAG+="uaccess"
    SUBSYSTEM=="usb", ATTRS{idVendor}=="0bb4", ATTRS{idProduct}=="2c87", TAG+="uaccess"
    # HTC Camera USB Node
    SUBSYSTEM=="usb", ATTRS{idVendor}=="114d", ATTRS{idProduct}=="8328", TAG+="uaccess"
    # HTC Mass Storage Node
    SUBSYSTEM=="usb", ATTRS{idVendor}=="114d", ATTRS{idProduct}=="8200", TAG+="uaccess"
    SUBSYSTEM=="usb", ATTRS{idVendor}=="114d", ATTRS{idProduct}=="8a12", TAG+="uaccess"

Note that if you are working in a VM then you might need to configure the VM host to automatically attach USB HID devices to the guest.

You will then need to install SteamVR to pair the trackers with their wireless USB adapters. If you don't have a HMD then you might need to add one or two lines to your SteamVR configuration so that you can run SteamVR without a HMD. I'm not going to cover how to pair the devices; suffice it to say that if you see a green light then you are paired correctly.

# Installing the low-level C driver

The low-level driver can in theory bind to an arbitrary number of trackers, directly over USB or through a USB wireless adapter (also referred to as a Watchman dongle). To compile the driver you will need a few things:

    sudo apt install build-essential libjson-c-dev zlib1g-dev libusb-1.0-0-dev git cmake

Now checkout and build the code in a directory of your choice

    git clone https://github.com/asymingt/libdeepdive.git
    cd deepdive
    mkdir build
    cd build
    cmake ..
    make -j2
    sudo make install

You should now be able to use the deepdive_tool to probe your devices. 

    Usage: deepdive_tool [-i01bth] [--help]
    This program extracts and prints data from a vive system.
      -i, --imu                 print imu
      -0, --axis0               print rotation about LH 0
      -1, --axis1               print rotation about LH 1
      -b, --button              print buttons
      -t, --tracker             print tracker info
      -h, --lh                  print lighthouse info
      --help                    print this help and exit

Note that deepdive does not begin streaming any lights data until a complete OOTX packet is received from a lighthouse. This is because the light cannot be corrected until the base station parameters are known. Try:

    deepdive_tool -h


# Installing the high-level ROS/C++ driver

You will first need to install the ros-kinetic-desktop package from [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). The installation requires a few steps and takes a fair amount of time. You will then also need to install ceres-solver, the Kinetic distribution of OpenCV 3 and catkin-tools:

    sudo apt install libceres-dev ros-kinetic-opencv3 python-catkin-tools

Once this is done, source your ROS distribution:

    source /opt/ros/kinetic/setup.bash

The high-level driver is most simply installed into a catkin workspace. Navigate to some working directory and then create this workspace.

    mkdir -p catkin_ws/src
    cd catkin_ws
    catkin init

Now, create a symbolic link from the 'ros' directory of the deepdive source code to a folder in the drc directory. 


    pushd src
    ln -s /path/to/libdeepdive/ros deepdive_ros
    popd src

Then, build the code. The build products will be put in a folder called ```build```, while the compiled artifacts will be put in a directory called ```devel```.

    catkin build

Now, source the compiled products:

    source devel/setup.bash

You are now ready to use the high-level driver.

# Overview the high-level driver

The high-level driver supports the notion of profiles through YAML configuration files. This allows one to save configuration bundles for different experimental contexts, and easily switch between them. Crucially, a profile describes a rigid body. All trackers listed in the profile are assumed to be rigidly attached to the rigid body being tracked,

Before discussing the configuration files it probably make sense to explain the layout of the folders in the ```ros``` subfolder.:

1. cal - Calibration info is stored in this directory
1. conf - YAML configurations are stored in this directory
1. data - Data bags are stored in this directory
1. perf - Performance statistics are stored in this directory
1. rviz - Visualization configuration files are stored in this directory.

WWorking with Vive requires an understanding of all frames that are used in the system. Tracking is achieved using this transform chain, which related the sensor pose in the light frame (x) to its pose in the lighthouse frame (y):

    y = lTv vTw wTb bTh hTl x

The observed light angles are simple trigonometric functions of y, perturbed by measurement noise

Where the transforms are defined as the following:
  
    lTv = vive to lighthouse frame [lighthouse calibration]
    vTw = world to vive frame [registration]
    wTb = vive to lighthouse frame [predicted by the tracking algorithm]
    bTh = head (bolt) to body frame [extrinsic calibration]
    hTl = light to head frame [read from JSON]

Similarly, 

    y = wTb bTh hTi x

Where 

    hTi = IMU to head frame [read from JSON]

There are three components to the high-level driver

1. deepdive_bridge - A proxy that invokes the low-level driver to pull raw light and IMU measurements, and forward them on the ROS messaging backbone, where they can be consumed by other nodes and/or saved to bag files.

2. deepdive_calibration - An algorithm for calculating the slave to master lighthouse pose using PNP / Kabsch, and optionally a vive to world transform that maps the local poses to some global reference frame. We call this single affine transform the "registration".

3. deepdive_refine - A non-linear least squares solver for jointly estimating the sensor trajectory, registration, the slave to master lighthouse transform, tracker locations (extrinsics)

4. deepdive_track - An Unscented Kalman Filter that tracks the latent pose of the body frame with respect to the world frame, as well as its first two derivatives, under the assumption that acceleration is constant. The state is used to predict IMU and light measurements in one of many tracker frames, and the residual error between these predictions and the observations (raw light and IMU measurements) is used to correct the state periodically.

# Example usage

## Step 1 : Create your YAML profile

First, you'll need to know a bit about your hardware. You can use deepdive_tool with the -h switch to get information about your tracker and lighthouses:

    deepdive_tool -h
    Read calibration data for tracker LHR-08DE963B
    Found watchman LHR-08DE963B
    Metadata received for lighthouse with serial  3097796425
    - Motor 0 calibration
      (PHASE) 0.051666
      (TILT) -0.004829
      (GIBPHASE) 3.691406
      (GIBMAG) 0.004978
      (CURVE) -0.001614
    - Motor 1 calibration
      (PHASE) 0.038116
      (TILT) -0.001799
      (GIBPHASE) 1.530273
      (GIBMAG) -0.005566
      (CURVE) -0.002436
    - Accel x = -6.000000 y = 127.000000 z = 86.000000
    Metadata received for lighthouse with serial  907388239
    - Motor 0 calibration
      (PHASE) 0.039459
      (TILT) -0.006325
      (GIBPHASE) 1.692383
      (GIBMAG) 0.003714
      (CURVE) 0.000282
    - Motor 1 calibration
      (PHASE) 0.016495
      (TILT) -0.003832
      (GIBPHASE) 0.875488
      (GIBMAG) -0.012169
      (CURVE) -0.002443
    - Accel x = 0.000000 y = 127.000000 z = 95.000000

Using this information you can now copy the example profile to a new YAML profile, inserting the serial numbers of your hardware. I'm going to assume you called this 'myprofile.yaml'.

    # List of lighthouses to be included in tracking
    lighthouses:
      - "lighthouse_left"
      - "lighthouse_right"
    lighthouse_left:
      serial:          "3097796425"
      transform:       [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.000]
    lighthouse_right:
      serial:          "907388239"
      transform:       [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.000]

    # List of trackers to be included in tracking (see blocks below)
    trackers:
      - "tracker_test"
    tracker_test:
      serial:          "LHR-08DE963B"
      extrinsics:      [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.000]

Note that I have set the extrinsics for the single tracker to the identity transform. This is the same as saying that the rigid body's origin coincides with the bolt on the underside of the tracker. If you have different extrinsics, then you will need to update this.

## Step 2 : Record some data

Now, run this command:

    roslaunch deepdive_ros record.launch profile:=myprofile bag:=first

This will launch the deepdive_bridge and and record all data from all trackers to a file called first.bag. If you didn't specify the bag name it would be default record it to myprofile.bag. Note that a tracker might take several tens of seconds to start getting light data. For this reason I typically open another terminal to see which trackers and lighthouses are seen:

    rostopic echo /light/header/frame_id
    rostopic echo /light/lighthouse

Now, do something interesting with the trackers. When you are finished, press ctrl+c to end. This will safely stop recording and shut down.

You can now get some information about how much data was recorded:

    rosbag info /path/to/deepdive/ros/data/first.bag

Also, note that you can use the ```compress``` utility to reduce bag sizes. This is not necessary, but helps if you are checking you bag into git:

    pushd /path/to/deepdive/ros/data
    rosbag compress first.bag
    rm -rf first.orig.bag
    popd

## Step 2 : Run calibration

You are now ready to calibrate your system. The offline flag tells it to replay data rather than launch the bridge. The speed flag increases the replay speed by five times, so that you get a result quicker.

    roslaunch deepdive_ros calibrate.launch profile:=myprofile bag:=first offline:= true speed:=5

If you collected some data and the calibration algorithm completed successfully, you should see a file myprofile.tf2 created in the cal folder of the ros subfolder. 

    0.0209715 -0.971985 -1.90025 -0.396829 -0.00849138 0.00564542 0.917836 world vive
    0 0 0 0 0 0 1 vive 3097796425
    0.72591 1.36228 1.64107 0.578008 -0.5109 0.248715 0.585686 vive 907388239
    0 0.1625 -0.202 0 1 0 6.12323e-17 body LHR-08DE963B

Each line is a transform of the form ```[x y z qx qy qz qw parent child]```. You might notice that some of the parent and child frames match the serial numbers of your hardware. You might also notice that the first lighthouse always has the identity transform to the vive frame; this is by design.

## Step 3 : Offline tracking

In many cases experimental scenarios it is OK to simply solve for the trajectory after the experiment is complete. This is useful if, for example, you wish to get a trace of how the rigid body moved or benchmark another localization system's performance.

To do this, type the following command:

    roslaunch deepdive_ros refine.launch profile:=myprofile bag:=first offline:=true speed:=5

By default, only the rigid body trajectory (wTb) is solved for, while everything else is held constant. You can change this in the YAML file.

    # What else to refine, besides the trajectory
    refine:
      trajectory:       true       # If false, see below...
      registration:     false      # Refine vive -> world transform
      lighthouses:      false      # Refine lighthouse -> vive transform
      extrinsics:       false      # If false tracker frame is the body frame
      sensors:          false      # Refine sensor positions
      params:           false      # Refine lighthouse calibration parameters
      head:             false      # Refine head transform

If you set the trajectory to false, it indicates that you have some other means of tracking the body frame (Vicon, etc). The refine code will look at TF2 for this data. Specifically, it will look for world -> body transforms being spat out by the other system. This is probably only useful to people who are tying to solve for lighthouse calibration parameters.

## Step 3 : Online tracking (not available yet)

You are welcome to look at the code, but it doesn't work yet.

# Notes

Although we know how to extract the base station correction parameters, we don't yet know the model for the calibration. For this the code does not yet correct for lighthouse errors. If you believe that you have the correct model, set the parameter 'correct' to 'true' in your YAML profile and update the ```Predict(...)``` function in 'deepdive.hh':

```c++
// Given a point in space, predict the lighthouse angle
template <typename T>
static void Predict(T const* params, T const* xyz, T* ang, bool correct) {
  if (correct) {
    ang[0] = atan2(xyz[0] - (params[0*NUM_PARAMS + PARAM_TILT] + params[0*NUM_PARAMS + PARAM_CURVE] * xyz[1]) * xyz[1], xyz[2]);
    ang[1] = atan2(xyz[1] - (params[1*NUM_PARAMS + PARAM_TILT] + params[1*NUM_PARAMS + PARAM_CURVE] * xyz[0]) * xyz[0], xyz[2]);
    ang[0] -= params[0*NUM_PARAMS + PARAM_PHASE] + params[0*NUM_PARAMS + PARAM_GIB_MAG] * sin(ang[0] + params[0*NUM_PARAMS + PARAM_GIB_PHASE]);
    ang[1] -= params[1*NUM_PARAMS + PARAM_PHASE] + params[1*NUM_PARAMS + PARAM_GIB_MAG] * sin(ang[1] + params[1*NUM_PARAMS + PARAM_GIB_PHASE]);
  } else {
    ang[0] = atan2(xyz[0], xyz[2]);
    ang[1] = atan2(xyz[1], xyz[2]);
  }
}
```


# Copying (MIT licence)

Code based on: https://github.com/cnlohr/libsurvive
Some documentation from: https://github.com/nairol/LighthouseRedox
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