The true distance between the lighthouses in granite-ekf.bag is 87.5 inches, or
2.2225 meters. The measurement is taken from the midpoint of the two plastic
covers through which the laster and sync pulses are transmitted.

To solve for the lighthous poses:

roslaunch deepdive_ros granite.launch offline:=true log:=screen
  replay:=/home/asymingt/projects/personal/libdeepdive/ros/data/granite-ekf.bag

> Distance between lighthouses 2935772913 and 668207140 is 2.22842 meters