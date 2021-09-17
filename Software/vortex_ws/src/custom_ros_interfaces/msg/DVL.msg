std_msgs/Header header

# difference in time between current & previous measurments.
float64 dt

# Measured velocity [m/s]
geometry_msgs/Twist  twist

# Velocity variance
float64  variance

# Dead reckoning from velocity [In DVL Frame]
geometry_msgs/Point translation

# Altitude of the sensor
float64 altitude

# Generalized confidence of measurments
float64 confidence

# Temprature warning status
bool status

# Count of measurments
float64 count


