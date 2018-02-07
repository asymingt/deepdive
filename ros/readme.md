This project provides a ROS interface to the HTC Vive tracking system. It is
comprised of three distrinct ROS nodes:

  1. deepdive_bridge - Proxy between ROS and the libdeepdive USB drivers. This node pulls raw IMU, light, button, tracker and lighthouse data from a USB or wireless Watchman dongle, and then publishes this data as ROS messages. We can then stream this data to bag files, or directly into the sensor fusion algorithms below.

  2. deepdive_solver - A ceres-solver based approach to jointly refining the world-to-lighthouse transforms, body-to-tracker transforms, photodiode extrinsics, and lighthouse calibration parameters. In order to do this, it also calculates
  the tracker poses. However, critically, it does not store a history of thes poses. Rather, 

  3. deepdive_filter - An Unscented Kalman Filter that fuses IMU measurements for a single tracker with light measurements to determine its pose.

# Quick guide

You need ROS Kinetic or Lunar and catkin_tools. Please refer to their online manuals to help with installation. You will then checkout, compile and source this code:

  source /opt/ros/<distro>/setup.bash
  mkdir -p ~/deepdive_ws/src
  cd ~/deepdive_ws/src
  git clone catkin_simple
  git clone libdeepdive
  catkin init
  catkin build
  source devel/setup.bash

Next, you must use the SteamVR application to pair all your trackers / controllers with wireless Watchman dongles. When they are correctly paired, the LEDs will be green. If they are blue, then they are unpaired. If on, pushing the button breaks the pairing. So, to turn off, you need to hold down the button until the blue light blinks.

This tool will help you enumerate all the trackers and lighthouses in your system. If you don't see any, make sure you have the Linux udev rules correctly setup. If you don't see any lighthouses, make sure they are turned on and facing the trackers.

  rosrun deepdive_ros deepdive_list

You now need to define which trackers you will be using. To do this, make a file called 
"alpha.yaml" in the cfg

  serial: "LHR-09DF88FD"
  frame: "alpha"
  gravity:          [0.0, -9.80665, 0.0]
  rate: 128
  initial_estimate:
    position:       [0, 0, 0]
    attitude:       [0, 0, 0, 1]
    velocity:       [0, 0, 0]
    gyro_bias:      [0, 0, 0]
  initial_covariance:
    position:       [100, 100, 100]
    attitude:       [10, 10, 10]
    velocity:       [1.0e-06, 1.0e-06, 1.0e-06]
    gyro_bias:      [1.0e-01, 1.0e-01, 1.0e-01]
  process_noise:
    position:       [0, 0, 0]
    attitude:       [0, 0, 0]
    velocity:       [0, 0, 0]
    gyro_bias:      [2.4e-10, 2.4e-10, 2.4e-10]

# Description

Robotics problem typically involve tracking several rigid bodies. 

The frame transform hierarchy looks something like this:

                    +-----------------------F-----------------------+
                    |                                               |
    SENSOR --O-- TRACKER --K-- HEAD --S-- BODY -(C)- WORLD --S-- LIGHTHOUSE
                    |
                    K
                    |
                   IMU

Where
  (K) are constant transforms extracted from the devices.
  (F) are transforms calculated by deepdive_filter
  (S) are transforms calculated by deepdive_solver
  (C) are transforms provided as corrections to the solver