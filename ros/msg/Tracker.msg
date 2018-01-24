string serial                           # Tracker serial number
deepdive_ros/Sensor[] sensors           # Sensor position/normals
geometry_msgs/Vector3 acc_bias          # Acceleromater bias
geometry_msgs/Vector3 acc_scale         # Accelerometer scale
geometry_msgs/Vector3 gyr_bias          # Gyro bias
geometry_msgs/Vector3 gyr_scale         # Gyro scale
geometry_msgs/Transform imu_transform   # IMU in tracking (photodiode) frame
geometry_msgs/Transform head_transform  # HEAD in tracking (photodiode) frame
uint32 lastcount                        # Last count
uint32 overflows                        # Number of overflows
