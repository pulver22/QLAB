# Command a percentage amount of total thrust capacity, and an turn rate.
# On a conventional craft, turn rate would map to rudder.

# Thrust amount ranges from [-1.0..1.0], where 1.0 pushes Kingfisher forward.
float32 thrust

# Yaw rate specified in radians/sec, where positive values cause Kingfisher
# to turn toward the port deck. The controller will use feedback from the
# IMU's gyroscopes to attempt to match the command rate.
float32 yaw_rate

