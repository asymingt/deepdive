string serial                         # Tracker serial number
deepdive_ros/Sensor[] sensors         # Sensor position/normals
geometry_msgs/Vector3 acc_bias        # Acceleromater bias
geometry_msgs/Vector3 acc_scale       # Accelerometer scale
geometry_msgs/Vector3 gyr_bias        # Gyro bias
geometry_msgs/Vector3 gyr_scale       # Gyro scale
geometry_msgs/Transform imu_transform # IMU to tracking frame transform
uint32 lastcount                      # Last time count
float64 tickstore                     # Number of timer overflows
